#include "Lander.h"

Lander::Lander() {
	angularThrust = 15;
	verticalThrust = 3;
	thrust = 2;
	velocity = glm::vec3(0, 0, 0);
	acceleration = glm::vec3(0, 0, 0);
	force = glm::vec3(0, 0, 0);
	mass = 1;
	damping = .99;
	angularForce = 0;
	angularVelocity = 0;
	angularAcceleration = 0;
	restitution = 0.5;
	//pos = getPosition();
	setRotation(0, 90, 0, 1, 0);
	rot = getRotationAngle(0);
	onGround = false;
}

void Lander::update(bool pause, map<int, bool> keymap, GravityForce gravityForce, TurbulenceForce turbForce, Box landerBounds, bool exploded) {
	bounds = landerBounds;
	gameOver = exploded;

	bool moveForward = keymap['w'] || keymap['W'];
	bool moveLeft = keymap['a'] || keymap['A'];
	bool moveBackward = keymap['s'] || keymap['S'];
	bool moveRight = keymap['d'] || keymap['D'];
	bool thrustUp = keymap[' '];
	bool rotateLeft = keymap['q'] || keymap['Q'];
	bool rotateRight = keymap['e'] || keymap['E'];


	glm::vec3 forwardVector = heading(true) * thrust;
	glm::vec3 sidewayVector = heading(false) * thrust;
	//glm::vec3 forceVector = glm::vec3(thrust, thrust, thrust);
	if (!gameOver) {
		if (moveForward) {
			force += forwardVector;
		}
		if (moveBackward) {
			force -= forwardVector;
		}
		if (moveLeft) {
			force += sidewayVector;
		}
		if (moveRight) {
			force -= sidewayVector;
		}
		if (rotateLeft) {
			angularForce += angularThrust;
		}
		if (rotateRight) {
			angularForce -= angularThrust;

		}
		if (thrustUp) {
			// play thrust sound
			force.y += verticalThrust;
			onGround = false;
		}
	}
	if (!onGround) {
		force += gravityForce.getForce();
	}
	force += turbForce.getForce();

	pauseIntegration = pause;
	if (!pauseIntegration) {
		integrate();
	}
	// reset forces
	force = glm::vec3(0, 0, 0);
	angularForce = 0;
}

void Lander::integrate() {
	float framerate = ofGetFrameRate();
	float dt = 1.0 / framerate;

	glm::vec3 newPos = glm::vec3(getPosition().x, getPosition().y, getPosition().z) + velocity * dt;
	setPosition(newPos.x, newPos.y, newPos.z);
	acceleration = glm::vec3(0, 0, 0);
	acceleration += (force * 1.0 / mass);
	velocity += acceleration * dt;
	velocity *= damping;
	//cout << glm::length(velocity) << endl;
	// prevent sliding/sinking
	if (onGround && glm::length(velocity) < 0.3) {
		velocity = glm::vec3(0, 0, 0);
	}

	setRotation(0, getRotationAngle(0) + angularVelocity * dt, 0, 1, 0);
	float a = angularAcceleration;
	a += (angularForce * 1.0 / mass);
	angularVelocity += a * dt;
	angularVelocity *= damping;
	// prevent being able to turn on ground
	if (onGround && angularVelocity < 0.1) {
		angularVelocity = 0;
	}
}

float Lander::resolveCollision(glm::vec3 normal, Octree& octree) {
	vector<Box> boxListRtn;
	glm::vec3 impulseForce = (1 + restitution) * (glm::dot(-velocity, normal) * normal);
	float sum = impulseForce.x + impulseForce.y + impulseForce.z;
	if (sum > 0.4) {
		float t = 0.01;
		while (octree.intersect(bounds, octree.root, boxListRtn)) {
			//	move along contact normal iteratively t = 0.01
			glm::vec3 newPos = glm::vec3(getPosition().x, getPosition().y, getPosition().z) + normal * t;
			setPosition(newPos.x, newPos.y, newPos.z);
			ofVec3f min = getSceneMin() + getPosition();
			ofVec3f max = getSceneMax() + getPosition();
			bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
		}
		force += impulseForce * ofGetFrameRate();
	}
	return sum;
}


float Lander::detectAltitude(Octree& octree) {
	ofPoint landerPos = getPosition();
	Ray ray = Ray(Vector3(landerPos.x, landerPos.y, landerPos.z), Vector3(landerPos.x, landerPos.y - 1000, landerPos.z));
	if (octree.intersect(ray, octree.root, selectedNode)) {
		// there is node below, we can get altitude
		//ofVec3f point = octree.mesh.getVertex(selectedNode.points[0]);
		Vector3 point = selectedNode.box.center();
		//octree.drawBox(selectedNode.box);
		//cout << selectedNode.box.center().y() << endl;
		return landerPos.y - point.y();
	}
	else {
		return NULL;
	}
}



glm::vec3 Lander::heading(bool straight) {
	float angle;
	if (straight) {
		angle = glm::radians(-getRotationAngle(0));
	}
	else {
		angle = glm::radians(-(getRotationAngle(0) + 90));
	}
	glm::vec3 head = glm::normalize(glm::vec3(glm::cos(angle), 0, glm::sin(angle)));
	return head;
}

void Lander::addForce(ParticleForce* f) {
	forces.push_back(f);
}