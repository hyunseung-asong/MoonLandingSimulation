#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxAssimpModelLoader.h"
#include "Octree.h"
#include <glm/gtx/intersect.hpp>
#include "Lander.h"
#include "Particle.h"
#include "ParticleEmitter.h"
#include "Gui.h"



class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent2(ofDragInfo dragInfo);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void drawAxis(ofVec3f);
		void initLightingAndMaterials();
		void savePicture();
		void toggleWireframeMode();
		void togglePointsDisplay();
		void toggleSelectTerrain();
		void setCameraTarget();
		bool mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point);
		bool raySelectWithOctree(ofVec3f &pointRet);
		glm::vec3 ofApp::getMousePointOnPlane(glm::vec3 p , glm::vec3 n);
		void loadVbo();

		ofEasyCam mainCam;
		ofCamera fixedCam;
		ofCamera topCam;
		ofCamera* currCam;
		ofxAssimpModelLoader moon;
		Lander lander;
		ofLight light;
		Box boundingBox, landerBounds;
		Box testBox;
		vector<Box> colBoxList;
		bool bLanderSelected = false;
		Octree octree;
		TreeNode selectedNode;
		glm::vec3 mouseDownPos, mouseLastPos;
		bool bInDrag = false;


		ofxIntSlider numLevels;
		ofxPanel settings;

		bool bAltKeyDown;
		bool bCtrlKeyDown;
		bool bWireframe;
		bool bDisplayPoints;
		bool bPointSelected;
		bool bHide;
		bool pointSelected = false;
		bool bDisplayLeafNodes = false;
		bool bDisplayOctree = false;
		bool bDisplayBBoxes = false;

		
		
		bool bLanderLoaded;
		bool bTerrainSelected;
	
		ofVec3f selectedPoint;
		ofVec3f intersectPoint;

		vector<Box> bboxList;

		const float selectionRange = 4.0;


		map<int, bool> keymap;
		GravityForce landerGravityForce;
		ofxFloatSlider landerGravitySlider;
		TurbulenceForce landerTurbulenceForce;
		ofxVec3Slider landerTurbMinSlider;
		ofxVec3Slider landerTurbMaxSlider;
		ofxToggle resolveCollisions;
		ofxToggle visualizeCollisions;

		ofImage background;
		float windowWidth;
		float windowHeight;

		float landerAltitude;
		bool altitudeSensor;
		Gui gui;
		glm::vec3 averageNormal;


		ParticleEmitter thrustEmitter;
		TurbulenceForce* thrustTurbForce;
		GravityForce* thrustGravityForce;
		ImpulseRadialForce* thrustRadialForce;
		CyclicForce* thrustCyclicForce;
		ofxFloatSlider thrustGravity;
		ofxFloatSlider thrustDamping;
		ofxFloatSlider thrustRadius;
		ofxVec3Slider thrustVelocity;
		ofxIntSlider thrustNumParticles;
		ofxFloatSlider thrustLifespan;
		ofxVec2Slider thrustLifespanRange;
		ofxVec3Slider thrustTurbMin;
		ofxVec3Slider thrustTurbMax;
		ofxFloatSlider thrustMass;
		ofxFloatSlider thrustRadialForceVal;
		ofxFloatSlider thrustRadialHight;
		ofxFloatSlider thrustCyclicForceVal;
		ofxFloatSlider thrustRate;
		ofxPanel thrustSettings;
		ofTexture particleTex;
		ofVbo vbo;
		ofShader shader;


		ParticleEmitter explosionEmitter;
		ParticleEmitter explosionRingEmitter;
		TurbulenceForce* explosionTurbForce;
		GravityForce* explosionGravityForce;
		ImpulseRadialForce* explosionRadialForce;
		ImpulseRadialForce* explosionRingForce;
		CyclicForce* explosionCyclicForce;
		ofxFloatSlider explosionGravity;
		ofxFloatSlider explosionDamping;
		ofxFloatSlider explosionRadius;
		ofxVec3Slider explosionVelocity;
		ofxIntSlider explosionNumParticles;
		ofxFloatSlider explosionLifespan;
		ofxVec2Slider explosionLifespanRange;
		ofxVec3Slider explosionTurbMin;
		ofxVec3Slider explosionTurbMax;
		ofxFloatSlider explosionMass;
		ofxFloatSlider explosionRadialForceVal;
		ofxFloatSlider explosionRingForceVal;
		ofxFloatSlider explosionRingHeight;
		ofxFloatSlider explosionCyclicForceVal;
		ofxFloatSlider explosionRate;
		ofxPanel explosionSettings;
		bool exploded = true;

		float lastShot = 0;

		ofLight baseLighting;
		ofLight landingSpotLight;
		ofLight landerLight;

		ofMaterial moonMaterial;
	

		ofSoundPlayer thrustSound;
		ofSoundPlayer explosionSound;
		bool pause = false;

		
};
