
#include "ofApp.h"
#include "Util.h"


//--------------------------------------------------------------
// setup scene, lighting, state and load geometry
//
void ofApp::setup() {

	ofSetWindowTitle("Final Project - 3D Landing Simulation");
	ofSetFrameRate(60);
	ofSetVerticalSync(true);
	windowWidth = ofGetWindowWidth();
	windowHeight = ofGetWindowHeight();
	background.load("images/StarrySpaceBackground.jpg");
	background.resize(windowWidth, windowHeight);

	bHide = false;
	bWireframe = false;
	bDisplayPoints = false;
	bAltKeyDown = false;
	bCtrlKeyDown = false;
	bLanderLoaded = false;
	bTerrainSelected = false;
	altitudeSensor = true;
	//	ofSetWindowShape(1024, 768);
	mainCam.setDistance(20);
	mainCam.setNearClip(.1);
	mainCam.setFov(65.5);   // approx equivalent to 28mm in 35mm format
	mainCam.disableMouseInput();
	ofEnableSmoothing();
	ofEnableDepthTest();
	//ofEnableLighting();
	ofNoFill();

	fixedCam.setGlobalPosition(glm::vec3(50, 5, 0));
	fixedCam.lookAt(glm::vec3(0, 0, 0));
	topCam.setGlobalPosition(glm::vec3(0, 150, 0));
	topCam.lookAt(glm::vec3(0, 0, 0));
	currCam = &mainCam;


	


	// setup rudimentary lighting 
	//
	initLightingAndMaterials();
	baseLighting.setup();
	baseLighting.enable();
	//baseLighting.setScale(5);
	baseLighting.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
	baseLighting.setDiffuseColor(ofFloatColor(0.5, 0.5, 0.5));
	//light2.setSpecularColor(ofFloatColor(1, 1, 1));
	baseLighting.rotate(-90, ofVec3f(1, 0, 0));
	baseLighting.setPosition(0, 100, 0);

	// lighting setup
	landingSpotLight.setup();
	//light1.disable();
	landingSpotLight.enable();
	landingSpotLight.setSpotlight();
	//landingSpotLight.setScale(.05);
	landingSpotLight.setSpotlightCutOff(30);
	landingSpotLight.setAttenuation(.2, .001, .001);
	landingSpotLight.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
	landingSpotLight.setDiffuseColor(ofFloatColor(0.7, 0.7, 0.7));
	landingSpotLight.setSpecularColor(ofFloatColor(1, 1, 1));
	landingSpotLight.rotate(-90, ofVec3f(1, 0, 0));
	landingSpotLight.setPosition(0, 50, 0);

	landerLight.setup();
	landerLight.setSpotlight();
	//landerLight.setScale(.05);
	landerLight.setSpotlightCutOff(30);
	landerLight.setAttenuation(.2, .001, .001);
	landerLight.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
	landerLight.setDiffuseColor(ofFloatColor(0.3, 0.3, 0.3));
	landerLight.setSpecularColor(ofFloatColor(0.3, 0.3, 0.3));
	landerLight.rotate(-90, ofVec3f(1, 0, 0));
	landerLight.disable();

	moonMaterial.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1, 1.0));
	moonMaterial.setDiffuseColor(ofFloatColor(0.3, 0.3, 0.3, 1.0));
	moonMaterial.setSpecularColor(ofFloatColor(0.8, 0.8, 0.8, 1.0));
	moonMaterial.setShininess(5);


	moon.loadModel("geo/moon-houdini.obj");
	moon.setScaleNormalization(false);

	// create sliders for testing
	//
	settings.setup();
	settings.add(numLevels.setup("Number of Octree Levels", 1, 1, 10));
	settings.add(landerGravitySlider.setup("Lander Gravity", 2, 0, 10));
	settings.add(landerTurbMinSlider.setup("Lander Turb Min", ofVec3f(0, 0, 0), ofVec3f(-5, -5, -5), ofVec3f(5, 5, 5)));
	settings.add(landerTurbMaxSlider.setup("Lander Turb Max", ofVec3f(0, 0, 0), ofVec3f(-5, -5, -5), ofVec3f(5, 5, 5)));
	settings.add(resolveCollisions.setup("Resolve Collisions", true));
	settings.add(visualizeCollisions.setup("Visualize Collisions", false));

	// setup lander forces
	landerGravityForce = GravityForce(ofVec3f(0, -landerGravitySlider, 0));
	landerTurbulenceForce = TurbulenceForce(ofVec3f(landerTurbMinSlider->x, landerTurbMinSlider->y, landerTurbMinSlider->z), ofVec3f(landerTurbMaxSlider->x, landerTurbMaxSlider->y, landerTurbMaxSlider->z));


	// sound loading
	thrustSound.load("sounds/thruster_sound_loop.wav");
	thrustSound.setVolume(0.1);
	thrustSound.setLoop(true);
	thrustSound.play();
	thrustSound.setPaused(true);
	explosionSound.load("sounds/explosion_sound.wav");
	explosionSound.setVolume(0.5);


	//texture loading
	ofDisableArbTex();
	if (!ofLoadImage(particleTex, "images/dot.png")) {
		cout << "Particle Texture File: images/dot.png not found" << endl;
		ofExit();
	}
	//shader loading
#ifdef TARGET_OPENGLES
	shader.load("shaders_gles/shader");
#else
	shader.load("shaders/shader");
#endif
	//emitterSettings.setup();
	settings.add(thrustSettings.setup());
	//	emitterSettings.add(emitterVelocity.setup("Initial Velocity", ofVec3f(0, 20, 0), ofVec3f(0, 0, 0), ofVec3f(100, 100, 100)));	
	//	emitterSettings.add(emitterLifespan.setup("Lifespan", 2.0, .1, 10.0));(
	thrustSettings.add(thrustNumParticles.setup("Number of Particles", 500, 0, 2000));
	thrustSettings.add(thrustLifespanRange.setup("Lifespan Range", ofVec2f(0.3, 0.75), ofVec2f(.1, .2), ofVec2f(3, 10)));
	thrustSettings.add(thrustMass.setup("Mass", 1, .1, 10));
	//	emitterSettings.add(emitterRate.setup("Rate", 1.0, .5, 60.0));
	thrustSettings.add(thrustDamping.setup("Damping", .97, .8, 1.0));
	thrustSettings.add(thrustGravity.setup("Gravity", 20, -10, 10));
	thrustSettings.add(thrustRadius.setup("Radius", 5, 1, 10));
	thrustSettings.add(thrustTurbMin.setup("Turbulence Min", ofVec3f(-1, -1, -1), ofVec3f(-20, -20, -20), ofVec3f(20, 20, 20)));
	thrustSettings.add(thrustTurbMax.setup("Turbulence Max", ofVec3f(1, 1, 1), ofVec3f(-20, -20, -20), ofVec3f(20, 20, 20)));
	thrustSettings.add(thrustRadialForceVal.setup("Radial Force", 40, 0, 500));
	thrustSettings.add(thrustRadialHight.setup("Radial Height", .25, 0, 1.0));
	thrustSettings.add(thrustCyclicForceVal.setup("Cyclic Force", 0, 10, 500));


	thrustTurbForce = new TurbulenceForce(ofVec3f(thrustTurbMin->x, thrustTurbMin->y, thrustTurbMin->z), ofVec3f(thrustTurbMax->x, thrustTurbMax->y, thrustTurbMax->z));
	thrustGravityForce = new GravityForce(ofVec3f(0, -thrustGravity, 0));
	thrustRadialForce = new ImpulseRadialForce(thrustRadialForceVal);
	thrustCyclicForce = new CyclicForce(thrustCyclicForceVal);
	
	thrustEmitter.sys->addForce(thrustTurbForce);
	thrustEmitter.sys->addForce(thrustGravityForce);
	thrustEmitter.sys->addForce(thrustRadialForce);
	//thrustEmitter.sys->addForce(thrustCyclicForce);

	thrustEmitter.setVelocity(ofVec3f(0, 0, 0));
	thrustEmitter.setOneShot(true);
	thrustEmitter.setEmitterType(RadialEmitter);
	thrustEmitter.setGroupSize(thrustNumParticles);
	thrustEmitter.setRandomLife(true);
	thrustEmitter.setLifespanRange(ofVec2f(thrustLifespanRange->x, thrustLifespanRange->y));



	//emitterSettings.setup();
	settings.add(explosionSettings.setup());
	//	emitterSettings.add(emitterVelocity.setup("Initial Velocity", ofVec3f(0, 20, 0), ofVec3f(0, 0, 0), ofVec3f(100, 100, 100)));	
	//	emitterSettings.add(emitterLifespan.setup("Lifespan", 2.0, .1, 10.0));(
	explosionSettings.add(explosionNumParticles.setup("Number of Particles", 5000, 0, 20000));
	explosionSettings.add(explosionLifespanRange.setup("Lifespan Range", ofVec2f(3, 5), ofVec2f(.1, .2), ofVec2f(3, 10)));
	explosionSettings.add(explosionMass.setup("Mass", 1, .1, 10));
	//	emitterSettings.add(emitterRate.setup("Rate", 1.0, .5, 60.0));
	explosionSettings.add(explosionDamping.setup("Damping", .99, .8, 1.0));
	explosionSettings.add(explosionGravity.setup("Gravity", 0, -10, 10));
	explosionSettings.add(explosionRadius.setup("Radius", 5, 1, 10));
	explosionSettings.add(explosionTurbMin.setup("Turbulence Min", ofVec3f(-1, -1, -1), ofVec3f(-20, -20, -20), ofVec3f(20, 20, 20)));
	explosionSettings.add(explosionTurbMax.setup("Turbulence Max", ofVec3f(1, 1, 1), ofVec3f(-20, -20, -20), ofVec3f(20, 20, 20)));
	explosionSettings.add(explosionRadialForceVal.setup("Radial Force", 2500, 0, 5000));
	explosionSettings.add(explosionRingForceVal.setup("Ring Force", 100, 0, 5000));
	explosionSettings.add(explosionRingHeight.setup("Ring Height", 1, 0, 1.0));
	explosionSettings.add(explosionCyclicForceVal.setup("Cyclic Force", 2, 0, 10));


	explosionTurbForce = new TurbulenceForce(ofVec3f(explosionTurbMin->x, explosionTurbMin->y, explosionTurbMin->z), ofVec3f(explosionTurbMax->x, explosionTurbMax->y, explosionTurbMax->z));
	explosionGravityForce = new GravityForce(ofVec3f(0, -explosionGravity, 0));
	explosionRadialForce = new ImpulseRadialForce(explosionRadialForceVal);
	explosionCyclicForce = new CyclicForce(explosionCyclicForceVal);
	explosionRingForce = new ImpulseRadialForce(explosionRingForceVal);

	
	explosionEmitter.sys->addForce(explosionTurbForce);
	explosionEmitter.sys->addForce(explosionGravityForce);
	explosionEmitter.sys->addForce(explosionRadialForce);
	explosionEmitter.sys->addForce(explosionCyclicForce);
	explosionRingEmitter.sys->addForce(explosionGravityForce);
	explosionRingEmitter.sys->addForce(explosionTurbForce);
	explosionRingEmitter.sys->addForce(explosionRingForce);
	explosionRingEmitter.sys->addForce(explosionCyclicForce);

	explosionEmitter.setVelocity(ofVec3f(0, 0, 0));
	explosionEmitter.setOneShot(true);
	explosionEmitter.setEmitterType(RadialEmitter);
	explosionEmitter.setGroupSize(explosionNumParticles);
	explosionEmitter.setRandomLife(true);
	explosionEmitter.setLifespanRange(ofVec2f(explosionLifespanRange->x, explosionLifespanRange->y));
	//explosionRingEmitter.setVelocity(ofVec3f(0, 0, 0));
	explosionRingEmitter.setOneShot(true);
	explosionRingEmitter.setEmitterType(RadialEmitter);
	explosionRingEmitter.setGroupSize(explosionNumParticles);
	explosionRingEmitter.setRandomLife(true);
	explosionRingEmitter.setLifespanRange(ofVec2f(explosionLifespanRange->x, explosionLifespanRange->y));





	//  Create Octree for testing.
	//

	// Timing the creation of Octree
	float before = ofGetElapsedTimeMillis();
	octree.create(moon.getMesh(0), 20);
	//octree.create(moon.getMesh(0), 10);
	float after = ofGetElapsedTimeMillis();
	//cout << "Time taken to create Octree: " << (after - before) / 1000 << " seconds" << endl;

	//cout << "Number of Verts: " << moon.getMesh(0).getNumVertices() << endl;

	testBox = Box(Vector3(3, 3, 0), Vector3(5, 5, 2));


	keymap = map<int, bool>();
	for (int i = 0; i < keymap.size(); i++) { keymap[i] = false; }
	gui = Gui();

	//lander.loadModel("geo/lander.obj");
	//bLanderLoaded = true;
	//lander.setScaleNormalization(false);
	//lander.setPosition(0, 10, 0);
	////cout << "number of meshes: " << lander.getNumMeshes() << endl;
	//bboxList.clear();
	//for (int i = 0; i < lander.getMeshCount(); i++) {
	//	bboxList.push_back(Octree::meshBounds(lander.getMesh(i)));
	//}
}

//--------------------------------------------------------------
// incrementally update scene (animation)
//
void ofApp::update() {
	// live update of emmitter parameters (with sliders)
	thrustEmitter.setVelocity(ofVec3f(thrustVelocity->x, thrustVelocity->y, thrustVelocity->z));
	thrustEmitter.setParticleRadius(thrustRadius);
	thrustEmitter.setLifespanRange(ofVec2f(thrustLifespanRange->x, thrustLifespanRange->y));
	thrustEmitter.setMass(thrustMass);
	thrustEmitter.setDamping(thrustDamping);
	thrustEmitter.setGroupSize(thrustNumParticles);
	thrustEmitter.setParticleRadius(thrustRadius);

	explosionEmitter.setVelocity(ofVec3f(explosionVelocity->x, explosionVelocity->y, explosionVelocity->z));
	explosionEmitter.setParticleRadius(explosionRadius);
	explosionEmitter.setLifespanRange(ofVec2f(explosionLifespanRange->x, explosionLifespanRange->y));
	explosionEmitter.setMass(explosionMass);
	explosionEmitter.setDamping(explosionDamping);
	explosionEmitter.setGroupSize(explosionNumParticles);
	explosionEmitter.setParticleRadius(explosionRadius);

	explosionRingEmitter.setParticleRadius(explosionRadius);
	explosionRingEmitter.setLifespanRange(ofVec2f(explosionLifespanRange->x, explosionLifespanRange->y));
	explosionRingEmitter.setMass(explosionMass);
	explosionRingEmitter.setDamping(explosionDamping);
	explosionRingEmitter.setGroupSize(explosionNumParticles);
	explosionRingEmitter.setParticleRadius(explosionRadius);

	// live update of forces  (with sliders)
	thrustGravityForce->set(ofVec3f(0, -thrustGravity, 0));
	thrustTurbForce->set(ofVec3f(thrustTurbMin->x, thrustTurbMin->y, thrustTurbMin->z), ofVec3f(thrustTurbMax->x, thrustTurbMax->y, thrustTurbMax->z));
	thrustRadialForce->set(thrustRadialForceVal);
	thrustRadialForce->setHeight(thrustRadialHight);
	thrustCyclicForce->set(thrustCyclicForceVal);

	explosionGravityForce->set(ofVec3f(0, -explosionGravity, 0));
	explosionTurbForce->set(ofVec3f(explosionTurbMin->x, explosionTurbMin->y, explosionTurbMin->z), ofVec3f(explosionTurbMax->x, explosionTurbMax->y, explosionTurbMax->z));
	explosionRadialForce->set(explosionRadialForceVal);
	//explosionRadialForce->setHeight(explosionRadialForceVal);
	explosionCyclicForce->set(explosionCyclicForceVal);
	explosionRingForce->set(explosionRingForceVal);
	explosionRingForce->setHeight(explosionRingHeight);

	// don't forget to update emitter
	thrustEmitter.update();
	explosionEmitter.update();
	explosionRingEmitter.update();


	landerGravityForce.set(ofVec3f(0, -landerGravitySlider, 0));
	landerTurbulenceForce.set((ofVec3f)landerTurbMinSlider, (ofVec3f)landerTurbMaxSlider);
	if (windowWidth != ofGetWindowWidth() || windowHeight != ofGetWindowHeight()) {
		windowWidth = ofGetWindowWidth();
		windowHeight = ofGetWindowHeight();
		background.resize(windowWidth, windowHeight);
	}
	if (bLanderLoaded) {
		
		ofPoint landerPos = lander.getPosition();
		ofVec3f min = lander.getSceneMin() + lander.getPosition();
		ofVec3f max = lander.getSceneMax() + lander.getPosition();
		landerBounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
		Vector3 center = landerBounds.center();
		float time = ofGetElapsedTimeMillis();
		float offset = 0.4;
		thrustEmitter.setPosition(ofPoint(landerPos.x, landerPos.y + offset, landerPos.z));
		explosionEmitter.setPosition(glm::vec3(center.x(), center.y(), center.z()));
		explosionRingEmitter.setPosition(glm::vec3(center.x(), center.y(), center.z()));
		float lightOffset = 5;
		landerLight.setPosition(ofPoint(landerPos.x, landerPos.y + lightOffset, landerPos.z));
		float rate = 15;
		if (keymap[' '] && !exploded) {
			thrustSound.setPaused(false);
			if ((time - lastShot) > (1000.0 / rate)) {
					thrustEmitter.sys->reset();
					thrustEmitter.start();
					lastShot = time;
				
			}
		}
		else {
			thrustSound.setPaused(true);
		}

		// get all colliding boxes
		lander.update(pause, keymap, landerGravityForce, landerTurbulenceForce, landerBounds, exploded);
		colBoxList.clear();
		lander.onGround = octree.intersect(landerBounds, octree.root, colBoxList);
		vector<TreeNode> nodeList;
		octree.intersectNodeList(landerBounds, octree.root, nodeList);
		averageNormal = octree.getAverageNormal(nodeList);
		if (resolveCollisions) {
			//bounce lander towards normal vector
			if (lander.onGround) {
				lander.resolveCollision(averageNormal, octree);
				//if landing speed to high, "explode"
				float maxLandingSpeed = 1.2;
				if (glm::length(lander.velocity) > maxLandingSpeed && !exploded) {
					explosionEmitter.sys->reset();
					explosionEmitter.start();
					explosionRingEmitter.sys->reset();
					explosionRingEmitter.start();
					explosionSound.play();
					//cout << "EXPLODE" << endl;
					exploded = true;
					//game end
				}
			}
		}
		
		//cout << landerPos.y << endl;
		topCam.setGlobalPosition(glm::vec3(landerPos.x, landerPos.y + 8.6, landerPos.z));
		// SET CAMERA ROTATION with lander rotation angle
		topCam.lookAt(glm::vec3(landerPos.x, landerPos.y, landerPos.z));
		fixedCam.lookAt(glm::vec3(landerPos.x, landerPos.y, landerPos.z));


		if (altitudeSensor) {
			landerAltitude = lander.detectAltitude(octree);
			//ofVec3f p;
			//bool somethingBelow = lander.raySelectWithOctree(octree, p);
			//cout << somethingBelow << ", P: " << p << endl;
			//cout << landerAltitude << endl;
		}


	}
	gui.update(landerAltitude, altitudeSensor, exploded, bLanderLoaded);
}
//--------------------------------------------------------------
void ofApp::draw() {
	loadVbo();
	// 
	//ofBackground(ofColor::black);
	//ofSetColor(255);
	glDepthMask(false);
	background.draw(0, 0);
	if (!bHide) settings.draw();
	gui.draw();

	glDepthMask(true);


	// this makes everything look glowy :)
	//
	ofEnableBlendMode(OF_BLENDMODE_ADD);
	ofEnablePointSprites();

	currCam->begin();
	ofPushMatrix();
	if (bWireframe) {                    // wireframe mode  (include axis)
		ofDisableLighting();
		ofSetColor(ofColor::slateGray);
		moon.drawWireframe();
		if (bLanderLoaded) {
			lander.drawWireframe();
			//if (!bTerrainSelected) drawAxis(lander.getPosition());
		}
		if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}
	else {
		ofEnableLighting();              // shaded mode
		moonMaterial.begin();
		moon.drawFaces();
		moonMaterial.end();
		ofMesh mesh;
		
		//landingSpotLight.draw();

		if (bLanderLoaded) {
			// draw particle emitter here..
			//
			//thrustEmitter.draw();

			
			ofSetColor(255, 100, 90);
			shader.begin();
			particleTex.bind();
			int total = thrustEmitter.sys->particles.size() + explosionEmitter.sys->particles.size() + explosionRingEmitter.sys->particles.size();
			vbo.draw(GL_POINTS, 0, total);

			particleTex.unbind();


			shader.end();

			ofDisablePointSprites();
			ofDisableBlendMode();
			ofEnableAlphaBlending();
			ofSetColor(255);



			lander.drawFaces();

			ofPoint landerPos = lander.getPosition();
			//ofSetColor(255, 0, 0);
			//ofDrawLine(glm::vec3(landerPos.x, landerPos.y, landerPos.z), glm::vec3(landerPos.x, landerPos.y - 50, landerPos.z));




			//if (!bTerrainSelected) drawAxis(lander.getPosition());
			if (bDisplayBBoxes) {
				ofNoFill();
				ofSetColor(ofColor::white);
				for (int i = 0; i < lander.getNumMeshes(); i++) {
					ofPushMatrix();
					ofMultMatrix(lander.getModelMatrix());
					ofRotate(-90, 1, 0, 0);
					Octree::drawBox(bboxList[i]);
					ofPopMatrix();
				}
			}

			if (bLanderSelected) {

				ofVec3f min = lander.getSceneMin() + lander.getPosition();
				ofVec3f max = lander.getSceneMax() + lander.getPosition();

				landerBounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
				ofSetColor(ofColor::white);
				Octree::drawBox(landerBounds);
			}
			if (visualizeCollisions) {
				// draw colliding boxes
				ofSetColor(255, 0, 0);
				for (int i = 0; i < colBoxList.size(); i++) {
					Octree::drawBox(colBoxList[i]);
				}

				// draw normal vector
				ofSetColor(0, 255, 0);
				glm::vec3 center = octree.getAverageBoxCenter(colBoxList);
				ofDrawLine(center, center + averageNormal * 10);
			}
		}
	}
	if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));



	if (bDisplayPoints) {                // display points as an option    
		glPointSize(3);
		ofSetColor(ofColor::green);
		moon.drawVertices();
	}

	// highlight selected point (draw sphere around selected point)
	//
	if (bPointSelected) {
		ofSetColor(ofColor::blue);
		ofDrawSphere(selectedPoint, .1);
	}


	// recursively draw octree
	//
	ofDisableLighting();
	int level = 0;
	//	ofNoFill();

	if (bDisplayLeafNodes) {
		octree.drawLeafNodes(octree.root);
		//cout << "num leaf: " << octree.numLeaf << endl;
	}
	else if (bDisplayOctree) {
		ofNoFill();
		ofSetColor(ofColor::white);
		octree.draw(numLevels, 0);
	}

	// if point selected, draw a sphere
	//
	if (pointSelected) {
		ofVec3f p = octree.mesh.getVertex(selectedNode.points[0]);
		ofVec3f d = p - mainCam.getPosition();
		ofSetColor(ofColor::lightGreen);
		ofDrawSphere(p, .02 * d.length());
	}



	ofPopMatrix();
	currCam->end();

	glDepthMask(false);
	if (!bHide) settings.draw();
	gui.draw();
	glDepthMask(true);
}


// 
// Draw an XYZ axis in RGB at world (0,0,0) for reference.
//
void ofApp::drawAxis(ofVec3f location) {

	ofPushMatrix();
	ofTranslate(location);

	ofSetLineWidth(1.0);

	// X Axis
	ofSetColor(ofColor(255, 0, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(1, 0, 0));


	// Y Axis
	ofSetColor(ofColor(0, 255, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 1, 0));

	// Z Axis
	ofSetColor(ofColor(0, 0, 255));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 0, 1));

	ofPopMatrix();
}


void ofApp::keyPressed(int key) {
	keymap[key] = true;
	switch (key) {
	case 'B':
	case 'b':
		bDisplayBBoxes = !bDisplayBBoxes;
		break;
	case 'C':
	case 'c':
		if (mainCam.getMouseInputEnabled()) mainCam.disableMouseInput();
		else mainCam.enableMouseInput();
		break;
	case 'F':
	case 'f':
		ofToggleFullscreen();
		break;
	case 'H':
	case 'h':
		toggleSelectTerrain();
		break;
	case 'L':
	case 'l':
		bHide = !bHide;
		break;
	case 'O':
	case 'o':
		bDisplayOctree = !bDisplayOctree;
		break;
	case 'r':
		mainCam.reset();
		break;
		//case 's':
		//	savePicture();
			//break;
	case 't':
		setCameraTarget();
		break;
	case 'u':
		break;
	case 'v':
		togglePointsDisplay();
		break;
	case 'V':
		break;
		//case 'w':
		//	toggleWireframeMode();
		//	break;
	case 'z':
	case 'Z':
		altitudeSensor = !altitudeSensor;
		break;
	case '0':
		pause = !pause;
		break;
	case '1':
		currCam = &mainCam;
		break;
	case '2':
		currCam = &fixedCam;
		break;
	case '3':
		currCam = &topCam;
		break;
	case '4':
		if (landerLight.getIsEnabled()) {
			landerLight.disable();
		}
		else { landerLight.enable(); }
	case OF_KEY_ALT:
		mainCam.enableMouseInput();
		bAltKeyDown = true;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = true;
		break;
	case OF_KEY_SHIFT:
		break;
	case OF_KEY_DEL:
		break;
	case OF_KEY_RETURN: // reset game
		if (bLanderLoaded) {
			exploded = false;
		}
		else {
			cout << "Please load the lander first." << endl;
		}
		break;
	default:
		break;
	}
}

void ofApp::toggleWireframeMode() {
	bWireframe = !bWireframe;
}

void ofApp::toggleSelectTerrain() {
	bTerrainSelected = !bTerrainSelected;
}

void ofApp::togglePointsDisplay() {
	bDisplayPoints = !bDisplayPoints;
}

void ofApp::keyReleased(int key) {
	keymap[key] = false;
	switch (key) {

	case OF_KEY_ALT:
		mainCam.disableMouseInput();
		bAltKeyDown = false;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = false;
		break;
	case OF_KEY_SHIFT:
		break;
	default:
		break;

	}
}



//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {


}


//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

	// if moving camera, don't allow mouse interaction
	//
	if (mainCam.getMouseInputEnabled()) return;

	// if moving camera, don't allow mouse interaction
//
	if (mainCam.getMouseInputEnabled()) return;

	// if rover is loaded, test for selection
	//
	if (bLanderLoaded) {
		glm::vec3 origin = mainCam.getPosition();
		glm::vec3 mouseWorld = mainCam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
		glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);

		ofVec3f min = lander.getSceneMin() + lander.getPosition();
		ofVec3f max = lander.getSceneMax() + lander.getPosition();

		landerBounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
		bool hit = landerBounds.intersect(Ray(Vector3(origin.x, origin.y, origin.z), Vector3(mouseDir.x, mouseDir.y, mouseDir.z)), 0, 10000);
		if (hit) {
			bLanderSelected = true;
			mouseDownPos = getMousePointOnPlane(lander.getPosition(), mainCam.getZAxis());
			mouseLastPos = mouseDownPos;
			bInDrag = true;
		}
		else {
			bLanderSelected = false;
		}
	}
	else {
		ofVec3f p;
		raySelectWithOctree(p);
	}
}

bool ofApp::raySelectWithOctree(ofVec3f& pointRet) {
	ofVec3f mouse(mouseX, mouseY);
	ofVec3f rayPoint = mainCam.screenToWorld(mouse);
	ofVec3f rayDir = rayPoint - mainCam.getPosition();
	rayDir.normalize();
	Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z),
		Vector3(rayDir.x, rayDir.y, rayDir.z));

	pointSelected = octree.intersect(ray, octree.root, selectedNode);

	if (pointSelected) {
		pointRet = octree.mesh.getVertex(selectedNode.points[0]);
	}
	return pointSelected;
}




//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

	// if moving camera, don't allow mouse interaction
	//
	if (mainCam.getMouseInputEnabled()) return;

	if (bInDrag) {

		glm::vec3 landerPos = lander.getPosition();

		glm::vec3 mousePos = getMousePointOnPlane(landerPos, mainCam.getZAxis());
		glm::vec3 delta = mousePos - mouseLastPos;

		landerPos += delta;
		lander.setPosition(landerPos.x, landerPos.y, landerPos.z);
		mouseLastPos = mousePos;

		//ofVec3f min = lander.getSceneMin() + lander.getPosition();
		//ofVec3f max = lander.getSceneMax() + lander.getPosition();

		//Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
		//colBoxList.clear();
		//octree.intersect(bounds, octree.root, colBoxList);

		// Measuring time taken for intersect:
		// 
		//float before = ofGetElapsedTimeMicros();
		//octree.intersect(bounds, octree.root, colBoxList);
		//float after = ofGetElapsedTimeMicros();
		//cout << "Time taken to intersect Octree: " << (after - before) << " microseconds" << endl;


		//if (bounds.overlap(testBox)) {
		//	cout << "overlap" << endl;
		//}
		//else {
		//	cout << "OK" << endl;
		//}


	}
	else {
		ofVec3f p;
		raySelectWithOctree(p);
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
	bInDrag = false;
}



// Set the camera to use the selected point as it's new target
//  
void ofApp::setCameraTarget() {

}


//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}



//--------------------------------------------------------------
// setup basic ambient lighting in GL  (for now, enable just 1 light)
//
void ofApp::initLightingAndMaterials() {

	static float ambient[] =
	{ .5f, .5f, .5, 1.0f };
	static float diffuse[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float position[] =
	{ 5.0, 5.0, 5.0, 0.0 };

	static float lmodel_ambient[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float lmodel_twoside[] =
	{ GL_TRUE };


	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position);


	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	//	glEnable(GL_LIGHT1);
	glShadeModel(GL_SMOOTH);
}

void ofApp::savePicture() {
	ofImage picture;
	picture.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
	picture.save("screenshot.png");
	cout << "picture saved" << endl;
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//
void ofApp::dragEvent2(ofDragInfo dragInfo) {

	ofVec3f point;
	mouseIntersectPlane(ofVec3f(0, 0, 0), mainCam.getZAxis(), point);
	if (lander.loadModel(dragInfo.files[0])) {
		lander.setScaleNormalization(false);
		//		lander.setScale(.1, .1, .1);
			//	lander.setPosition(point.x, point.y, point.z);
		lander.setPosition(1, 1, 0);

		bLanderLoaded = true;
		exploded = false;
		for (int i = 0; i < lander.getMeshCount(); i++) {
			bboxList.push_back(Octree::meshBounds(lander.getMesh(i)));
		}

		cout << "Mesh Count: " << lander.getMeshCount() << endl;
	}
	else cout << "Error: Can't load model" << dragInfo.files[0] << endl;
}

bool ofApp::mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f& point) {
	ofVec2f mouse(mouseX, mouseY);
	ofVec3f rayPoint = mainCam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
	ofVec3f rayDir = rayPoint - mainCam.getPosition();
	rayDir.normalize();
	return (rayIntersectPlane(rayPoint, rayDir, planePoint, planeNorm, point));
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//
void ofApp::dragEvent(ofDragInfo dragInfo) {
	if (lander.loadModel(dragInfo.files[0])) {
		bLanderLoaded = true;
		exploded = false;
		lander.setScaleNormalization(false);
		lander.setPosition(0, 0, 0);
		cout << "number of meshes: " << lander.getNumMeshes() << endl;
		bboxList.clear();
		for (int i = 0; i < lander.getMeshCount(); i++) {
			bboxList.push_back(Octree::meshBounds(lander.getMesh(i)));
		}

		//		lander.setRotation(1, 180, 1, 0, 0);

				// We want to drag and drop a 3D object in space so that the model appears 
				// under the mouse pointer where you drop it !
				//
				// Our strategy: intersect a plane parallel to the camera plane where the mouse drops the model
				// once we find the point of intersection, we can position the lander/lander
				// at that location.
				//

				// Setup our rays
				//
		glm::vec3 origin = mainCam.getPosition();
		glm::vec3 camAxis = mainCam.getZAxis();
		glm::vec3 mouseWorld = mainCam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
		glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
		float distance;

		bool hit = glm::intersectRayPlane(origin, mouseDir, glm::vec3(0, 0, 0), camAxis, distance);
		if (hit) {
			// find the point of intersection on the plane using the distance 
			// We use the parameteric line or vector representation of a line to compute
			//
			// p' = p + s * dir;
			//
			glm::vec3 intersectPoint = origin + distance * mouseDir;

			// Now position the lander's origin at that intersection point
			//
			glm::vec3 min = lander.getSceneMin();
			glm::vec3 max = lander.getSceneMax();
			float offset = (max.y - min.y) / 2.0;
			//lander.setPosition(intersectPoint.x, intersectPoint.y - offset, intersectPoint.z
			lander.setPosition(intersectPoint.x, intersectPoint.y, intersectPoint.z);

			// set up bounding box for lander while we are at it
			//
			landerBounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
		}
	}


}

//  intersect the mouse ray with the plane normal to the camera 
//  return intersection point.   (package code above into function)
//
glm::vec3 ofApp::getMousePointOnPlane(glm::vec3 planePt, glm::vec3 planeNorm) {
	// Setup our rays
	//
	glm::vec3 origin = mainCam.getPosition();
	glm::vec3 camAxis = mainCam.getZAxis();
	glm::vec3 mouseWorld = mainCam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
	glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
	float distance;

	bool hit = glm::intersectRayPlane(origin, mouseDir, planePt, planeNorm, distance);

	if (hit) {
		// find the point of intersection on the plane using the distance 
		// We use the parameteric line or vector representation of a line to compute
		//
		// p' = p + s * dir;
		//
		glm::vec3 intersectPoint = origin + distance * mouseDir;

		return intersectPoint;
	}
	else return glm::vec3(0, 0, 0);
}


// load vertex buffer in preparation for rendering
//
void ofApp::loadVbo() {
	int totalSizes = thrustEmitter.sys->particles.size() + explosionEmitter.sys->particles.size() + explosionRingEmitter.sys->particles.size();


	if (totalSizes < 1) return;

	vector<ofVec3f> sizes;
	vector<ofVec3f> points;
	for (int i = 0; i < thrustEmitter.sys->particles.size(); i++) {
		points.push_back(thrustEmitter.sys->particles[i].position);
		sizes.push_back(ofVec3f(thrustRadius));
	}
	for (int i = 0; i < explosionEmitter.sys->particles.size(); i++) {
		points.push_back(explosionEmitter.sys->particles[i].position);
		sizes.push_back(ofVec3f(explosionRadius));
	}
	for (int i = 0; i < explosionRingEmitter.sys->particles.size(); i++) {
		points.push_back(explosionRingEmitter.sys->particles[i].position);
		sizes.push_back(ofVec3f(explosionRadius));
	}
	// upload the data to the vbo
	//
	int total = (int)points.size();
	vbo.clear();
	vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
	vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
}