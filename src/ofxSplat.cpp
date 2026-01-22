#include "ofxSplat.h"
#include "ply.h"
#include <cstdint> // For uint32_t and uint16_t
#include <cstring> // For memcpy
#include <cmath>
#include <vector>
#include <string>

// radiance / color stuff passed through
// with indices now, the number of points could be reduced from 6 to 4 per splat

//--------------------------------------------------------------
void ofxSplat::setup(string pointCloud){
    const static std::string shaderPath("../../../../../addons/ofxSplat/shaders/");
	shader.load(shaderPath+"vert.glsl", shaderPath+"frag.glsl");
    shader.bindDefaults();

    const std::string& filename = ofToDataPath(pointCloud);
     ply::PlyFile ply(filename);

     // Create accessors
     const auto x = ply.accessor<float>("x");
     const auto y = ply.accessor<float>("y");
     const auto z = ply.accessor<float>("z");
     const auto opacity = ply.accessor<float>("opacity");
     const auto scale_0 = ply.accessor<float>("scale_0");
     const auto scale_1 = ply.accessor<float>("scale_1");
     const auto scale_2 = ply.accessor<float>("scale_2");
     const auto rot_qw = ply.accessor<float>("rot_0");
     const auto rot_qx = ply.accessor<float>("rot_1");
     const auto rot_qy = ply.accessor<float>("rot_2");
     const auto rot_qz = ply.accessor<float>("rot_3");
     // Spherical harmonics accessors
     const auto f_dc_0 = ply.accessor<float>("f_dc_0");
     const auto f_dc_1 = ply.accessor<float>("f_dc_1");
     const auto f_dc_2 = ply.accessor<float>("f_dc_2");
    
    const size_t maxRestCoeffs = 45;
    std::vector<std::pair<size_t, ply::PlyAccessor<float>>> restAccessors;
    restAccessors.reserve(maxRestCoeffs);
    for (size_t i = 0; i < maxRestCoeffs; ++i) {
        const std::string name = "f_rest_" + std::to_string(i);
        if (ply.hasAccessor(name)) {
            restAccessors.emplace_back(i, ply.accessor<float>(name));
        }
    }
    const size_t restCoeffCount = restAccessors.size();
    const size_t perColorCoeff = std::min<size_t>(15, restCoeffCount / 3);
    if (restCoeffCount % 3 != 0) {
        ofLogWarning("ofxSplat") << "Unexpected f_rest count: " << restCoeffCount;
    }
    if (perColorCoeff >= 15) {
        shDegree = 3;
    } else if (perColorCoeff >= 8) {
        shDegree = 2;
    } else if (perColorCoeff >= 3) {
        shDegree = 1;
    } else {
        shDegree = 0;
    }
     int vertsRemoved = 0;
    for (size_t row = 0; row < ply.num_vertices(); ++row) {
        
        VertexData temp{};
        for (const auto& rest : restAccessors) {
            temp.f_rest[rest.first] = rest.second(row);
        }
				//            if(row%100000 == 0){
//                cout << "next:";
//                cout << temp.f_rest[i];
//            }
           
        temp.x = x(row);
        temp.y = y(row);
        temp.z = z(row);

//        if (row == 0){
//            cout << temp.x << " " << temp.y << " " << temp.z << endl;
//        }
        temp.opacity = 1.f / (1.f + std::exp(-opacity(row)));
        temp.scale[0] = exp(scale_0(row));
        temp.scale[1] = exp(scale_1(row));
        temp.scale[2] = exp(scale_2(row));
        
        float qlen = sqrt( pow(rot_qx(row), 2) +
                           pow(rot_qy(row), 2) +
                           pow(rot_qz(row), 2) +
                           pow(rot_qw(row), 2));
        
        temp.rot[0] = (rot_qw(row)/qlen);
        temp.rot[1] = (rot_qx(row)/qlen);
        temp.rot[2] = (rot_qy(row)/qlen);
        temp.rot[3] = (rot_qz(row)/qlen);
        
        if (true){
            // Create a quaternion from these components
            glm::quat quaternion(rot_qx(row),
                                 rot_qy(row),
                                 rot_qz(row),
                                 rot_qw(row));
            
            // Normalize the quaternion
            glm::quat normalizedQuaternion = glm::normalize(quaternion);
            
            // Scale and offset
            glm::vec4 scaled = (glm::vec4(normalizedQuaternion.x, normalizedQuaternion.y, normalizedQuaternion.z, normalizedQuaternion.w) * 128.0f) + 128.0f;
            
            // Clip the values to ensure they are within the 0-255 range
            glm::vec4 clipped;
            clipped.x = std::min(std::max(scaled.x, 0.0f), 255.0f);
            clipped.y = std::min(std::max(scaled.y, 0.0f), 255.0f);
            clipped.z = std::min(std::max(scaled.z, 0.0f), 255.0f);
            clipped.w = std::min(std::max(scaled.w, 0.0f), 255.0f);
            
            // Convert to bytes
            std::vector<uint8_t> bytes = {
                static_cast<uint8_t>(clipped.z),
                static_cast<uint8_t>(clipped.w),
                static_cast<uint8_t>(clipped.x),
                static_cast<uint8_t>(clipped.y)
            };
            

            temp.rot[0] = (float)(bytes[0]-128)/128.0;
            temp.rot[1] = (float)(bytes[1]-128)/128.0;
            temp.rot[2] = (float)(bytes[2]-128)/128.0;
            temp.rot[3] = (float)(bytes[3]-128)/128.0;
        }

        temp.f_dc[0] = f_dc_0(row);
        temp.f_dc[1] = f_dc_1(row);
        temp.f_dc[2] = f_dc_2(row);
        
        vertices.push_back(temp);

    }
    
    // subtract the center
    ofPoint center;
    for (int i = 0; i < vertices.size(); i++) {
        center +=ofPoint(vertices[i].x, vertices[i].y, vertices[i].z);
    }
    center/= (float)vertices.size();
    ofMatrix4x4 rot;
    for (int i = 0; i < vertices.size(); i++) {
        vertices[i].x -= center.x;
        vertices[i].y -= center.y;
        vertices[i].z -= center.z;
    }
    vector < float > data;
    
    for (size_t i = 0; i < vertices.size(); ++i) {
        
    
            const auto& v = vertices[i];

            float r = v.f_dc[0];
            float g =  v.f_dc[1];
            float b = v.f_dc[2];
            float a = v.opacity; // Opacity converted to alpha

        std::vector<double> rot = { v.rot[0], v.rot[1], v.rot[2], v.rot[3] };
        std::vector<double> scale = { v.scale[0], v.scale[1], v.scale[2] };
        std::vector<double> M(9, 0.0); // 9 elements for a 3x3 matrix
           M[0] = 1.0 - 2.0 * (rot[2] * rot[2] + rot[3] * rot[3]);
           M[1] = 2.0 * (rot[1] * rot[2] + rot[0] * rot[3]);
           M[2] = 2.0 * (rot[1] * rot[3] - rot[0] * rot[2]);
           M[3] = 2.0 * (rot[1] * rot[2] - rot[0] * rot[3]);
           M[4] = 1.0 - 2.0 * (rot[1] * rot[1] + rot[3] * rot[3]);
           M[5] = 2.0 * (rot[2] * rot[3] + rot[0] * rot[1]);
           M[6] = 2.0 * (rot[1] * rot[3] + rot[0] * rot[2]);
           M[7] = 2.0 * (rot[2] * rot[3] - rot[0] * rot[1]);
           M[8] = 1.0 - 2.0 * (rot[1] * rot[1] + rot[2] * rot[2]);

           // Apply scaling
        
           for (int j = 0; j < 9; ++j) {
               M[j] *= scale[j / 3];
               
               // I found printing this kind of stuff out super helpful to compare with the javascript code:
               
//               if (i == 0){
//                   cout << "M : " << M[j] << endl;
//               }

           }

           // Calculate sigma
           std::vector<float> sigma(6, 0.0);
           sigma[0] = M[0] * M[0] + M[3] * M[3] + M[6] * M[6];
           sigma[1] = M[0] * M[1] + M[3] * M[4] + M[6] * M[7];
           sigma[2] = M[0] * M[2] + M[3] * M[5] + M[6] * M[8];
           sigma[3] = M[1] * M[1] + M[4] * M[4] + M[7] * M[7];
           sigma[4] = M[1] * M[2] + M[4] * M[5] + M[7] * M[8];
           sigma[5] = M[2] * M[2] + M[5] * M[5] + M[8] * M[8];

        
        // helpful to debug
//        if (i == 0){
//            cout << sigma[0] << endl;
//            cout << sigma[1] << endl;
//            cout << sigma[2] << endl;
//            cout << sigma[3] << endl;
//            cout << sigma[4] << endl;
//            cout << sigma[5] << endl;
//        }
        
        std::vector<float> customData = {
            v.x, v.y, v.z, r, g, b, a, sigma[0],
            sigma[1], sigma[2], sigma[3]
        };
        for (int coeff = 0; coeff < 15; ++coeff) {
            float rCoeff = 0.0f;
            float gCoeff = 0.0f;
            float bCoeff = 0.0f;
            if (static_cast<size_t>(coeff) < perColorCoeff) {
                rCoeff = v.f_rest[coeff];
                gCoeff = v.f_rest[coeff + perColorCoeff];
                bCoeff = v.f_rest[coeff + perColorCoeff * 2];
            }
            customData.push_back(rCoeff);
            customData.push_back(gCoeff);
            customData.push_back(bCoeff);
        }
        customData.push_back(sigma[4]);
        customData.push_back(sigma[5]);
        customData.push_back(0.0f);

        for (auto & f : customData){
            data.push_back(f);
        }
        
    }

    mesh.setMode(OF_PRIMITIVE_TRIANGLES);
    
    vector < unsigned int > indices;
    for (int i = 0; i < vertices.size(); i++) {
        
        mesh.addVertex(ofPoint(-2, -2));
        mesh.addVertex(ofPoint(2, -2));
        mesh.addVertex(ofPoint(2, 2));
        
        
        mesh.addVertex(ofPoint(-2, -2));
        mesh.addVertex(ofPoint(-2, 2));
        mesh.addVertex(ofPoint(2, 2));
       
        mesh.addColor(ofFloatColor( 1, 0, 1) );
        mesh.addColor(ofFloatColor( 1, 0, 1) );
        mesh.addColor(ofFloatColor( 1, 0, 1) );
        
        mesh.addColor(ofFloatColor( 1, 0, 1) );
        mesh.addColor(ofFloatColor( 1, 0, 1) );
        mesh.addColor(ofFloatColor( 1, 0, 1) );
        
        indices.push_back(i * 6 + 0);
        indices.push_back(i * 6 + 1);
        indices.push_back(i * 6 + 2);
        indices.push_back(i * 6 + 3);
        indices.push_back(i * 6 + 4);
        indices.push_back(i * 6 + 5);
        

    }
    
    mesh.addIndices(indices);
    
    
    if (splatDataBuffer == 0) {
        glGenBuffers(1, &splatDataBuffer);
    }
    if (splatDataTex == 0) {
        glGenTextures(1, &splatDataTex);
    }
    glBindBuffer(GL_TEXTURE_BUFFER, splatDataBuffer);
    glBufferData(GL_TEXTURE_BUFFER, data.size() * sizeof(float), data.data(), GL_STATIC_DRAW);
    glBindTexture(GL_TEXTURE_BUFFER, splatDataTex);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, splatDataBuffer);
    glBindTexture(GL_TEXTURE_BUFFER, 0);
    glBindBuffer(GL_TEXTURE_BUFFER, 0);
    
    mesh.enableIndices();
}

//--------------------------------------------------------------
void ofxSplat::update(){

    if (ofGetFrameNum() % 60 == 0){
        shader.load("vert.glsl", "frag.glsl");
        shader.bindDefaults();
    }
}

//--------------------------------------------------------------
void ofxSplat::draw(){
    
   
    cam.begin();
    
	//use shader program
    ofDisableDepthTest();
	ofEnableAlphaBlending();
    //ofBackground(0);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  // Clear
    
    glBlendFuncSeparate(
        GL_ONE_MINUS_DST_ALPHA,
        GL_ONE,
        GL_ONE_MINUS_DST_ALPHA,
        GL_ONE
    );
    glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
 
	shader.begin();

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, splatDataTex);
    shader.setUniform1i("splatData", 0);
    shader.setUniform1i("splatStride", splatDataStride);
    
    shader.setUniform1i("splatCount", static_cast<int>(vertices.size()));
    shader.setUniformMatrix4f("view",cam.getModelViewMatrix());
    shader.setUniform2f("viewport", ofGetWidth(), ofGetHeight());
	shader.setUniformMatrix4f("projection", cam.getProjectionMatrix());
    shader.setUniform1f("time", ofGetElapsedTimef());
    float fov = cam.getFov();
    float viewportWidth = ofGetViewportWidth();
    float viewportHeight = ofGetViewportHeight();

    float focalLengthX = viewportWidth / (2.0f * tan(ofDegToRad(fov) / 2.0f));
    float focalLengthY = viewportHeight / (2.0f * tan(ofDegToRad(fov) / 2.0f));
    
    // I'm not convinced this is right:
	//shader.setUniform2f("focal", focalLengthX, focalLengthY);
    shader.setUniform2f("focal", 1159.5880733038064, 1164.6601287484507);

    shader.setUniform3f("cam_pos", cam.getPosition());
    shader.setUniform1i("sh_degree", shDegree);
    
    
    
    struct vertex2 {
        ofPoint pt;
        int index;
        int32_t depth;
    };

    vector < vertex2 > vertexsss;
    ofMatrix4x4 projview =  cam.getProjectionMatrix() * cam.getModelViewMatrix();
   
    
    int32_t maxDepth = std::numeric_limits<int32_t>::min();
    int32_t minDepth = std::numeric_limits<int32_t>::max();

    for (int i = 0; i < vertices.size(); i++) {
        vertex2 v;
        v.pt = ofPoint(vertices[i].x, vertices[i].y, vertices[i].z);
        v.index = i;
        

        v.depth =((projview(0, 2) * v.pt.x +
         projview(1, 2) * v.pt.y +
            projview(2, 2) * v.pt.z) * 4096);

        maxDepth = std::max(maxDepth, v.depth);
        minDepth = std::min(minDepth, v.depth);

        vertexsss.push_back(v);
    }

    // Inline lambda for counting sort
    auto countingSort = [&vertexsss, maxDepth, minDepth]() {
        int vertexCount = vertexsss.size();
        double depthRange = static_cast<double>(maxDepth) - minDepth;
        double depthInv = (depthRange > 0) ? ((256.0 * 256.0 - 1) / depthRange) : 1.0;
        
        std::vector<uint32_t> counts0(256 * 256, 0);
        
        // First pass: count occurrences
        for (int i = 0; i < vertexCount; i++) {
            int32_t normalizedDepth = static_cast<int32_t>((vertexsss[i].depth - minDepth) * depthInv);
            normalizedDepth = std::max(0, std::min(normalizedDepth, 256 * 256 - 1)); // Clamp value
            counts0[normalizedDepth]++;
        }

        std::vector<uint32_t> starts0(256 * 256, 0);
        uint32_t totalCount = 0;
        for (int i = 0; i < 256 * 256; i++) {
            starts0[i] = totalCount;
            totalCount += counts0[i];
        }

        std::vector<uint32_t> depthIndex(vertexCount);
        for (int i = 0; i < vertexCount; i++) {
            int32_t normalizedDepth = static_cast<int32_t>((vertexsss[i].depth - minDepth) * depthInv);
            normalizedDepth = std::max(0, std::min(normalizedDepth, 256 * 256 - 1)); // Clamp value
            
            if (starts0[normalizedDepth] < vertexCount) {
                depthIndex[starts0[normalizedDepth]++] = i;
            } else {
                std::cout << "Error: starts0[" << normalizedDepth << "] = " << starts0[normalizedDepth]
                          << " is out of bounds for depthIndex of size " << vertexCount << std::endl;
                // As a fallback, place this vertex at the end
                depthIndex[vertexCount - 1] = i;
            }
        }

        std::vector<vertex2> sortedVertices(vertexCount);
        for (int i = 0; i < vertexCount; i++) {
            sortedVertices[i] = vertexsss[depthIndex[i]];
        }

        vertexsss = std::move(sortedVertices);
    };

    // Call the counting sort
    countingSort();

    
    
    vector < unsigned int > indices;
    for (int i = 0; i < vertexsss.size(); i++) {
        
        int index = vertexsss[i].index;
        indices.push_back(index*6 + 0);
        indices.push_back(index*6 + 1);
        indices.push_back(index*6 + 2);
        indices.push_back(index*6 + 3);
        indices.push_back(index*6 + 4);
        indices.push_back(index*6 + 5);
    }
    
    mesh.getVbo().updateIndexData(indices.data(), indices.size());
    
	mesh.draw();
	shader.end();
    glBindTexture(GL_TEXTURE_BUFFER, 0);

    cam.end();
}
