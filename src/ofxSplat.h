#pragma once

#include "ofMain.h"
#include "ply.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>


#ifndef splat_h
#define splat_h

struct VertexData {
    float x, y, z;
    float opacity;
    float scale[3];
    float rot[4];
    float f_dc[3];
    float f_rest[15*3];
};



class ofxSplat {

    public:


		void setup(string pointCloud);
		void update();
		void draw();
		~ofxSplat();
        ofVboMesh mesh;

		ofShader shader;
        ofEasyCam cam;
        glm::vec3 sh[15];
        float camAngle =0;
        int shDegree = 0;

        vector < VertexData > vertices;
        GLuint splatDataBuffer = 0;
        GLuint splatDataTex = 0;
        int splatDataStride = 59;

	private:
		// Thread infrastructure
		std::thread sortWorker;
		std::mutex sortMutex;
		std::condition_variable sortCV;
		std::atomic<bool> sortReady{false};
		std::atomic<bool> sortRequested{false};
		std::atomic<bool> stopRequested{false};
		bool threadRunning = false;

		// Sort data structures (triple buffer)
		struct SortData {
			vector<unsigned int> indices;
			ofMatrix4x4 projViewMatrix;
			int32_t maxDepth;
			int32_t minDepth;
			size_t vertexCount;
		};

		SortData sortBuffers[3];
		int readBufferIndex = 0;
		int writeBufferIndex = 1;
		int pendingBufferIndex = 2;

		// Depth calculation results
		struct DepthVertex {
			int index;
			int32_t depth;
		};
		vector<DepthVertex> depthVertices;
		vector<DepthVertex> pendingDepthVertices;

		// Helper methods
		void sortThreadLoop();
		void performCountingSort();
		void calculateDepths(const ofMatrix4x4& projView);
		void swapSortBuffers();
		bool hasCameraChanged(const ofMatrix4x4& newMatrix, float threshold);
		void stopThread();

};
#endif /* splat_h */
