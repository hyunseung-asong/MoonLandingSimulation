#pragma once
#include "ofMain.h"
#include "ofxAssimpModelLoader.h"
#include "ParticleSystem.h"
#include "Octree.h"

class Lander : public ofxAssimpModelLoader {
public:
	Lander();
	void update(bool pause, map<int, bool> keymap, GravityForce gravityForce, TurbulenceForce turbulenceForce, Box landerBounds, bool exploded);
	void integrate();
	glm::vec3 heading(bool straight);
	void addForce(ParticleForce* f);
	float detectAltitude(Octree& octree);
	float resolveCollision(glm::vec3 normal, Octree& octree);

	vector<ParticleForce*> forces;
	float angularThrust;
	float thrust;
	float verticalThrust;
	glm::vec3 velocity;
	glm::vec3 acceleration;
	glm::vec3 force;
	float mass;
	float damping;
	float angularForce;
	float angularVelocity;
	float angularAcceleration;
	float restitution;
	glm::vec3 pos;
	float rot;
	glm::mat4 T;

	Box bounds;
	TreeNode selectedNode;
	bool pointSelected;
	bool onGround;
	bool gameOver;
	bool pauseIntegration;
};