// Vroom.js physics engine script (core classes only)
// This file contains the physics engine: vectors, rigid bodies, particles, springs, cloth, collision detection/resolution, and the physics world.
// No Three.js, UI, or rendering code included.

class VroomVector3 {
    constructor(x = 0, y = 0, z = 0) {
        this.x = x; this.y = y; this.z = z;
    }
    set(x, y, z) { this.x = x; this.y = y; this.z = z; return this; }
    copy(v) { this.x = v.x; this.y = v.y; this.z = v.z; return this; }
    add(v) { this.x += v.x; this.y += v.y; this.z += v.z; return this; }
    sub(v) { this.x -= v.x; this.y -= v.y; this.z -= v.z; return this; }
    multiply(s) { this.x *= s; this.y *= s; this.z *= s; return this; }
    dot(v) { return this.x * v.x + this.y * v.y + this.z * v.z; }
    length() { return Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z); }
    normalize() { const len = this.length(); if(len>0) this.multiply(1/len); return this; }
    distanceTo(v) { const dx=this.x-v.x, dy=this.y-v.y, dz=this.z-v.z; return Math.sqrt(dx*dx+dy*dy+dz*dz); }
    clone() { return new VroomVector3(this.x, this.y, this.z); }
}

// Rigid body class (spheres, cubes, arrows represented here)
class VroomRigidBody {
    constructor(meshPlaceholder = null, options = {}) {
        this.mesh = meshPlaceholder;
        this.id = Math.random().toString(36).substr(2,9);

        this.mass = options.mass || 1.0;
        this.inverseMass = this.mass > 0 ? 1 / this.mass : 0;
        this.density = options.density || 1.0;
        this.restitution = options.restitution || 0.7;
        this.staticFriction = options.staticFriction || 0.6;
        this.dynamicFriction = options.dynamicFriction || 0.4;
        this.rollingFriction = options.rollingFriction || 0.02;
        this.airDrag = options.airDrag || 0.001;
        this.radius = options.radius || options.size || 0.5;
        this.size = options.size || 1.0;
        this.shapeType = options.shapeType || 'sphere';

        this.position = new VroomVector3(options.x || 0, options.y || 0, options.z || 0);
        this.previousPosition = this.position.clone();
        this.velocity = new VroomVector3();
        this.previousVelocity = new VroomVector3();
        this.acceleration = new VroomVector3();
        this.force = new VroomVector3();
        this.angularVelocity = new VroomVector3();
        this.angularAcceleration = new VroomVector3();
        this.torque = new VroomVector3();

        this.momentOfInertia = this.calculateMomentOfInertia();
        this.inverseMomentOfInertia = this.momentOfInertia > 0 ? 1 / this.momentOfInertia : 0;

        this.isGrounded = false;
        this.groundContactTime = 0;
        this.isStatic = options.isStatic || false;
        this.sleepThreshold = 0.005;
        this.isSleeping = false;
        this.sleepTimer = 0;
        this.contactPoints = [];

        this.material = options.material || 'default';
        this.surfaceRoughness = options.surfaceRoughness || 0.5;

        this.kineticEnergy = 0;
        this.potentialEnergy = 0;
        this.rotationalEnergy = 0;

        this.centerOfMass = new VroomVector3(0,0,0);
        this.linearDamping = options.linearDamping || 0.01;
        this.angularDamping = options.angularDamping || 0.05;

        this.collisionHistory = [];
        this.maxCollisionHistory = 10;
    }

    calculateMomentOfInertia() {
        if (this.shapeType === 'sphere') {
            return (2/5) * this.mass * this.radius * this.radius;
        }
        return 1.0;
    }

    applyForce(force) {
        if (!this.isStatic) {
            this.checkForceWakeUp(force);
            this.force.add(force);
        }
    }

    applyImpulse(impulse) {
        if (!this.isStatic) {
            if (this.isSleeping && impulse.length() > 0.05) this.wakeUp('impulse');
            const scaled = impulse.clone().multiply(this.inverseMass);
            this.velocity.add(scaled);
        }
    }

    applyTorque(torque) {
        if (!this.isStatic) this.torque.add(torque);
    }

    applyAngularImpulse(angularImpulse) {
        if (!this.isStatic) {
            const scaled = angularImpulse.clone().multiply(this.inverseMomentOfInertia);
            this.angularVelocity.add(scaled);
        }
    }

    integrate(deltaTime) {
        if (this.isStatic) return;
        if (this.isSleeping) { this.checkPositionWakeUp(); return; }

        this.previousPosition.copy(this.position);
        this.previousVelocity.copy(this.velocity);

        // acceleration from forces
        this.acceleration.copy(this.force).multiply(this.inverseMass);
        this.angularAcceleration.copy(this.torque).multiply(this.inverseMomentOfInertia);

        // simple RK4-ish integration (keeps behavior deterministic and stable)
        const k1v = this.acceleration.clone().multiply(deltaTime);
        const k1p = this.velocity.clone().multiply(deltaTime);

        const k2v = this.acceleration.clone().multiply(deltaTime);
        const k2p = this.velocity.clone().add(k1v.clone().multiply(0.5)).multiply(deltaTime);

        const k3v = this.acceleration.clone().multiply(deltaTime);
        const k3p = this.velocity.clone().add(k2v.clone().multiply(0.5)).multiply(deltaTime);

        const k4v = this.acceleration.clone().multiply(deltaTime);
        const k4p = this.velocity.clone().add(k3v).multiply(deltaTime);

        this.velocity.add(k1v.add(k2v.multiply(2)).add(k3v.multiply(2)).add(k4v).multiply(1/6));
        this.position.add(k1p.add(k2p.multiply(2)).add(k3p.multiply(2)).add(k4p).multiply(1/6));

        const ka1 = this.angularAcceleration.clone().multiply(deltaTime);
        const ka2 = this.angularAcceleration.clone().multiply(deltaTime);
        const ka3 = this.angularAcceleration.clone().multiply(deltaTime);
        const ka4 = this.angularAcceleration.clone().multiply(deltaTime);

        this.angularVelocity.add(ka1.add(ka2.multiply(2)).add(ka3.multiply(2)).add(ka4).multiply(1/6));

        // update energies
        this.updateEnergyState();

        // clear forces/torques
        this.force.set(0,0,0);
        this.torque.set(0,0,0);

        // damping
        this.angularVelocity.multiply(Math.exp(-this.angularDamping * deltaTime));
        this.velocity.multiply(1 - this.linearDamping * deltaTime);

        // sleep management
        this.updateSleepState(deltaTime);
    }

    updateEnergyState() {
        this.kineticEnergy = 0.5 * this.mass * this.velocity.length() * this.velocity.length();
        this.rotationalEnergy = 0.5 * this.momentOfInertia * this.angularVelocity.length() * this.angularVelocity.length();
        this.potentialEnergy = this.mass * 9.81 * this.position.y;
    }

    updateSleepState(deltaTime) {
        const velocityMagnitude = this.velocity.length();
        const angularVelocityMagnitude = this.angularVelocity.length();
        const totalEnergy = this.kineticEnergy + this.rotationalEnergy;

        const isLowLinearEnergy = velocityMagnitude < this.sleepThreshold;
        const isLowAngularEnergy = angularVelocityMagnitude < this.sleepThreshold * 2;
        const isLowTotalEnergy = totalEnergy < this.sleepThreshold * this.mass;
        const isStable = this.isGrounded || this.position.y < 0.1;
        const hasLowAcceleration = this.acceleration.length() < 0.1;

        const positionChange = this.position.distanceTo(this.previousPosition);
        const isStationary = positionChange < 0.001;

        const canSleep = isLowLinearEnergy && isLowAngularEnergy && isLowTotalEnergy && isStable && hasLowAcceleration && isStationary;

        if (canSleep && !this.isSleeping) {
            this.sleepTimer += deltaTime;
            if (this.sleepTimer > 1.0) this.enterSleep();
        } else if (!canSleep && this.isSleeping) {
            this.wakeUp();
        } else if (!canSleep) {
            this.sleepTimer = 0;
        }

        this.isNearSleep = this.sleepTimer > 0.5 && !this.isSleeping;
    }

    enterSleep() {
        this.isSleeping = true;
        this.sleepStartTime = performance.now();
        if (this.velocity.length() < this.sleepThreshold) this.velocity.set(0,0,0);
        if (this.angularVelocity.length() < this.sleepThreshold*2) this.angularVelocity.set(0,0,0);
        this.force.set(0,0,0); this.torque.set(0,0,0);
        this.acceleration.set(0,0,0); this.angularAcceleration.set(0,0,0);
        this.sleepPosition = this.position.clone();
    }

    wakeUp(reason='unknown') {
        if (!this.isSleeping) return;
        this.isSleeping = false;
        this.isNearSleep = false;
        this.sleepTimer = 0;
        this.wakeUpReason = reason;
        this.wakeUpTime = performance.now();
    }

    checkForceWakeUp(appliedForce) {
        if (this.isSleeping && appliedForce.length() > 0.1) this.wakeUp('force');
    }

    checkPositionWakeUp() {
        if (this.isSleeping && this.sleepPosition) {
            const displacement = this.position.distanceTo(this.sleepPosition);
            if (displacement > 0.01) this.wakeUp('displacement');
        }
    }
}

// Cloth particle for Verlet integration
class VroomParticle {
    constructor(position, mass = 0.1, isFixed = false) {
        this.position = position.clone();
        this.previousPosition = position.clone();
        this.velocity = new VroomVector3();
        this.force = new VroomVector3();
        this.mass = mass;
        this.inverseMass = mass > 0 ? 1 / mass : 0;
        this.isFixed = isFixed;
        this.tempPosition = position.clone();
    }
    applyForce(f) { if (!this.isFixed) this.force.add(f); }
    integrate(deltaTime, gravity) {
        if (this.isFixed) return;
        // Verlet integration
        const acceleration = gravity.clone().add(this.force.clone().multiply(this.inverseMass));
        const nextPosition = this.position.clone().multiply(2).sub(this.previousPosition).add(acceleration.multiply(deltaTime * deltaTime));
        this.previousPosition.copy(this.position);
        this.position.copy(nextPosition);
        this.force.set(0,0,0);
    }
}

// Spring (constraint) connecting two particles
class VroomSpring {
    constructor(p1, p2, restLength, stiffness, type='structural') {
        this.p1 = p1; this.p2 = p2; this.restLength = restLength; this.stiffness = stiffness; this.type = type;
    }
    constrain(iterations = 1) {
        for (let i=0;i<iterations;i++) {
            const delta = this.p2.position.clone().sub(this.p1.position);
            const distance = delta.length();
            if (distance < 1e-6) continue;
            const diff = (distance - this.restLength) / distance;
            const correction = delta.multiply(diff * 0.5 * this.stiffness);

            const w1 = this.p1.inverseMass;
            const w2 = this.p2.inverseMass;
            const totalWeight = w1 + w2;
            if (totalWeight > 0) {
                if (!this.p1.isFixed) this.p1.position.add(correction.clone().multiply(w1 / totalWeight));
                if (!this.p2.isFixed) this.p2.position.sub(correction.clone().multiply(w2 / totalWeight));
            }
        }
    }
}

// Cloth simulation with triangle-sphere collision resolution
class VroomCloth {
    constructor(width, height, resolutionX, resolutionY, particleMass = 0.05, stiffness = 0.6, damping = 0.1) {
        this.resolutionX = resolutionX;
        this.resolutionY = resolutionY;
        this.particleMass = particleMass;
        this.stiffness = stiffness;
        this.damping = damping;
        this.particles = [];
        this.springs = [];
        this.mesh = null;
        this.width = width;
        this.height = height;
        this.initStructure();
    }

    clothThickness = 0.02;
    clothCollisionResponse = 0.22;

    getParticle(x,y) { return this.particles[y * this.resolutionX + x]; }

    initStructure() {
        for (let j=0;j<this.resolutionY;j++) {
            for (let i=0;i<this.resolutionX;i++) {
                const x = (i / (this.resolutionX - 1)) * this.width - this.width / 2;
                const y = 8 + j * 0.1;
                const z = (j / (this.resolutionY - 1)) * this.height - this.height / 2;
                const pos = new VroomVector3(x,y,z);
                this.particles.push(new VroomParticle(pos, this.particleMass, false));
            }
        }
        const spacingX = this.width / (this.resolutionX - 1);
        const spacingY = this.height / (this.resolutionY - 1);

        for (let j=0;j<this.resolutionY;j++) {
            for (let i=0;i<this.resolutionX;i++) {
                const p = this.getParticle(i,j);
                if (i < this.resolutionX - 1) this.addSpring(p, this.getParticle(i+1,j), spacingX, this.stiffness * 1.0, 'structural');
                if (j < this.resolutionY - 1) this.addSpring(p, this.getParticle(i,j+1), spacingY, this.stiffness * 1.0, 'structural');
                if (i < this.resolutionX - 1 && j < this.resolutionY - 1) {
                    this.addSpring(p, this.getParticle(i+1,j+1), Math.sqrt(spacingX*spacingX + spacingY*spacingY), this.stiffness * 0.8, 'shear');
                }
                if (i > 0 && j < this.resolutionY - 1) {
                    this.addSpring(p, this.getParticle(i-1,j+1), Math.sqrt(spacingX*spacingX + spacingY*spacingY), this.stiffness * 0.8, 'shear');
                }
                if (i < this.resolutionX - 2) this.addSpring(p, this.getParticle(i+2,j), spacingX*2, this.stiffness * 0.2, 'bend');
                if (j < this.resolutionY - 2) this.addSpring(p, this.getParticle(i,j+2), spacingY*2, this.stiffness * 0.2, 'bend');
            }
        }
    }

    addSpring(p1,p2,restLength,stiffness,type){ this.springs.push(new VroomSpring(p1,p2,restLength,stiffness,type)); }

    step(deltaTime, gravity, rigidBodies) {
        for (const particle of this.particles) particle.integrate(deltaTime, gravity);

        const constraintIterations = 4;
        for (let k=0;k<constraintIterations;k++) {
            for (const spring of this.springs) spring.constrain();
            this.resolveCollisions(rigidBodies);
        }

        this.applyDamping(this.damping * deltaTime);
        // Mesh update hooks are left to renderer integration; here we only update internal positions.
    }

    applyDamping(dampingFactor) {
        for (const particle of this.particles) {
            if (particle.isFixed) continue;
            const currentVelocity = particle.position.clone().sub(particle.previousPosition);
            currentVelocity.multiply(1 - dampingFactor);
            particle.previousPosition.copy(particle.position.clone().sub(currentVelocity));
        }
    }

    _crossProduct(a, b) {
        return new VroomVector3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }

    _closestPointToTriangle(p1, p2, p3, target) {
        const ab = p2.clone().sub(p1);
        const ac = p3.clone().sub(p1);
        const ap = target.clone().sub(p1);

        const d1 = ab.dot(ap);
        const d2 = ac.dot(ap);

        const edge1 = p2.clone().sub(p1);
        const edge2 = p3.clone().sub(p1);
        let normal = this._crossProduct(edge1, edge2).normalize();

        if (d1 <= 0 && d2 <= 0) return { point: p1.clone(), normal };

        const bp = target.clone().sub(p2);
        const d3 = ab.dot(bp);
        const d4 = ac.dot(bp);
        if (d3 >= 0 && d4 <= d3) return { point: p2.clone(), normal };

        const vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) {
            const v = d1 / (d1 - d3);
            return { point: p1.clone().add(ab.multiply(v)), normal };
        }

        const cp = target.clone().sub(p3);
        const d5 = ab.dot(cp);
        const d6 = ac.dot(cp);
        if (d6 >= 0 && d5 <= d6) return { point: p3.clone(), normal };

        const vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) {
            const w = d2 / (d2 - d6);
            return { point: p1.clone().add(ac.multiply(w)), normal };
        }

        const va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
            const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            const edge_p2p3 = p3.clone().sub(p2);
            return { point: p2.clone().add(edge_p2p3.multiply(w)), normal };
        }

        const denom = va + vb + vc;
        const v = vb / denom;
        const w = vc / denom;
        const closestPoint = p1.clone().add(edge1.multiply(v)).add(edge2.multiply(w));
        return { point: closestPoint, normal };
    }

    _resolveTriangleSphereCollision(p1, p2, p3, body) {
        const sphereCenter = body.position;
        const sphereRadius = body.radius;
        const thickness = this.clothThickness;
        const totalRadius = sphereRadius + thickness * 0.5;

        const { point: closestPoint, normal: triNormal } = this._closestPointToTriangle(
            p1.position, p2.position, p3.position, sphereCenter
        );

        const sphereToClosest = sphereCenter.clone().sub(closestPoint);
        const distance = sphereToClosest.length();
        if (distance >= totalRadius || distance < 1e-6) return false;

        let collisionNormal = sphereToClosest.clone().normalize();
        if (collisionNormal.length() < 1e-6) {
            collisionNormal = triNormal.clone().multiply(sphereToClosest.dot(triNormal) > 0 ? 1 : -1);
        }

        const penetration = totalRadius - distance;
        const correction = collisionNormal.clone().multiply(penetration * 1.01);

        const w1 = p1.inverseMass * this.clothCollisionResponse;
        const w2 = p2.inverseMass * this.clothCollisionResponse;
        const w3 = p3.inverseMass * this.clothCollisionResponse;
        const wB = body.inverseMass;
        const totalWeight = w1 + w2 + w3 + wB;
        if (totalWeight < 1e-6) return false;
        const invTotalWeight = 1 / totalWeight;

        if (!body.isStatic) {
            const bodyCorrection = correction.clone().multiply(wB * invTotalWeight);
            body.position.add(bodyCorrection);
            if (body.isSleeping) body.wakeUp('cloth_collision');
        }

        if (!p1.isFixed) p1.position.sub(correction.clone().multiply(w1 * invTotalWeight));
        if (!p2.isFixed) p2.position.sub(correction.clone().multiply(w2 * invTotalWeight));
        if (!p3.isFixed) p3.position.sub(correction.clone().multiply(w3 * invTotalWeight));

        return true;
    }

    resolveCollisions(rigidBodies) {
        const groundLevel = 0;
        for (const particle of this.particles) {
            if (particle.isFixed) continue;
            if (particle.position.y < groundLevel) {
                particle.position.y = Math.max(groundLevel + 0.001, particle.position.y);
                const velX = particle.position.x - particle.previousPosition.x;
                const velY = particle.position.y - particle.previousPosition.y;
                const velZ = particle.position.z - particle.previousPosition.z;

                const restThreshold = 0.003;
                if (Math.abs(velY) < restThreshold) {
                    particle.previousPosition.x = particle.position.x;
                    particle.previousPosition.y = particle.position.y;
                    particle.previousPosition.z = particle.position.z;
                } else {
                    if (velY < 0) {
                        const restitutionFactor = 0.5;
                        particle.previousPosition.y = particle.position.y + velY * Math.min(Math.max(restitutionFactor * 0.2, -1), 1);
                    }
                }

                const lateralDamping = 0.6;
                particle.previousPosition.x = particle.position.x - (particle.position.x - particle.previousPosition.x) * (1 - lateralDamping);
                particle.previousPosition.z = particle.position.z - (particle.position.z - particle.previousPosition.z) * (1 - lateralDamping);
            }

            if (!isFinite(particle.position.x) || !isFinite(particle.position.y) || !isFinite(particle.position.z)) {
                particle.position.set(0, groundLevel + 0.1, 0);
                particle.previousPosition.copy(particle.position);
            } else {
                const maxCoord = 1e3;
                particle.position.x = Math.min(Math.max(particle.position.x, -maxCoord), maxCoord);
                particle.position.y = Math.min(Math.max(particle.position.y, -maxCoord), maxCoord);
                particle.position.z = Math.min(Math.max(particle.position.z, -maxCoord), maxCoord);
            }
        }

        // Triangle collision requires an indexed triangle layout; cloth geometry consumer should set this.geometry with index array of triangles
        if (!this.geometry || !this.geometry.index) return;
        const indices = this.geometry.index.array;
        const numTriangles = indices.length / 3;

        for (const body of rigidBodies) {
            if (body.isStatic || body.isSleeping) continue;
            const considerAsTipSphere = (body.shapeType === 'sphere' || body.shapeType === 'arrow');
            if (!considerAsTipSphere) continue;

            for (let t=0;t<numTriangles;t++) {
                const i1 = indices[t*3], i2 = indices[t*3+1], i3 = indices[t*3+2];
                const p1 = this.particles[i1], p2 = this.particles[i2], p3 = this.particles[i3];

                if (body.shapeType === 'arrow') {
                    // consumer must provide body.arrow.tipLocal and body.mesh quaternion if using this feature
                    const tipLocal = body.arrow && body.arrow.tipLocal ? body.arrow.tipLocal : new VroomVector3(body.length * 0.5,0,0);
                    // If consumer supplies a function to get world tip position, use it; otherwise skip detailed arrow sampling.
                    if (typeof body.getTipWorld === 'function') {
                        const tipWorld = body.getTipWorld();
                        const tipProxy = { position: tipWorld, radius: body.radius, inverseMass: body.inverseMass, isStatic: body.isStatic, isSleeping: body.isSleeping };
                        const collided = this._resolveTriangleSphereCollision(p1,p2,p3,tipProxy);
                        if (collided && !body.isStatic) {
                            body.position.copy(tipWorld.clone());
                            body.velocity.multiply(0.08);
                            if (!body.arrow) body.arrow = {};
                            body.arrow.embed = true;
                            body.arrow.embedDepth = Math.min(0.2, (body.arrow.embedDepth || 0) + 0.05);
                        }
                    }
                } else {
                    this._resolveTriangleSphereCollision(p1,p2,p3,body);
                }
            }
        }
    }

    // Helper to let external code set triangle indices for cloth (since rendering geometry is external)
    setGeometryIndex(indexArray) {
        this.geometry = { index: { array: indexArray } };
    }
}

// Collision detector utilities
class VroomCollisionDetector {
    static sphereToSphere(b1,b2) {
        const distance = b1.position.distanceTo(b2.position);
        const minDistance = b1.radius + b2.radius;
        if (distance < minDistance && distance > 0.001) {
            return { hasCollision:true, normal: b2.position.clone().sub(b1.position).normalize(), penetration: minDistance - distance, contactPoint: b1.position.clone().add(b2.position.clone().sub(b1.position).multiply(0.5)) };
        }
        return { hasCollision:false };
    }

    static sphereToCube(sphere, cube) {
        const half = cube.size * 0.5;
        const closest = new VroomVector3(
            Math.max(cube.position.x - half, Math.min(sphere.position.x, cube.position.x + half)),
            Math.max(cube.position.y - half, Math.min(sphere.position.y, cube.position.y + half)),
            Math.max(cube.position.z - half, Math.min(sphere.position.z, cube.position.z + half))
        );
        const distance = sphere.position.distanceTo(closest);
        if (distance < sphere.radius) {
            const normal = sphere.position.clone().sub(closest).normalize();
            return { hasCollision:true, normal, penetration: sphere.radius - distance, contactPoint: closest };
        }
        return { hasCollision:false };
    }

    static cubeToCube(b1,b2) {
        const half1 = b1.size * 0.5, half2 = b2.size * 0.5;
        const dx = Math.abs(b1.position.x - b2.position.x);
        const dy = Math.abs(b1.position.y - b2.position.y);
        const dz = Math.abs(b1.position.z - b2.position.z);
        if (dx < (half1 + half2) && dy < (half1 + half2) && dz < (half1 + half2)) {
            const penetrationX = (half1 + half2) - dx;
            const penetrationY = (half1 + half2) - dy;
            const penetrationZ = (half1 + half2) - dz;
            let normal, penetration;
            if (penetrationX < penetrationY && penetrationX < penetrationZ) {
                normal = new VroomVector3(b1.position.x > b2.position.x ? 1 : -1, 0, 0); penetration = penetrationX;
            } else if (penetrationY < penetrationZ) {
                normal = new VroomVector3(0, b1.position.y > b2.position.y ? 1 : -1, 0); penetration = penetrationY;
            } else {
                normal = new VroomVector3(0, 0, b1.position.z > b2.position.z ? 1 : -1); penetration = penetrationZ;
            }
            return { hasCollision:true, normal, penetration, contactPoint: b1.position.clone().add(b2.position).multiply(0.5) };
        }
        return { hasCollision:false };
    }

    static detectCollision(b1,b2) {
        if (b1.shapeType === 'sphere' && b2.shapeType === 'sphere') return this.sphereToSphere(b1,b2);
        if (b1.shapeType === 'sphere' && b2.shapeType === 'cube') return this.sphereToCube(b1,b2);
        if (b1.shapeType === 'cube' && b2.shapeType === 'sphere') {
            const col = this.sphereToCube(b2,b1);
            if (col.hasCollision) col.normal.multiply(-1);
            return col;
        }
        if (b1.shapeType === 'cube' && b2.shapeType === 'cube') return this.cubeToCube(b1,b2);
        return { hasCollision:false };
    }

    static sphereToPlane(body, planeY) {
        if (body.shapeType === 'sphere') {
            const distance = body.position.y - planeY;
            const radius = body.radius;
            if (distance < radius) return { hasCollision:true, normal: new VroomVector3(0,1,0), penetration: radius - distance, contactPoint: new VroomVector3(body.position.x, planeY, body.position.z) };
        } else if (body.shapeType === 'cube') {
            const cubeBottom = 0; // consumer may provide more precise bottom offset
            const actualBottomY = body.position.y + cubeBottom;
            const distance = actualBottomY - planeY;
            if (distance < 0.02) return { hasCollision:true, normal: new VroomVector3(0,1,0), penetration: Math.abs(distance), contactPoint: new VroomVector3(body.position.x, planeY, body.position.z) };
        }
        return { hasCollision:false };
    }
}

// Collision resolver (position correction, impulses, friction)
class VroomCollisionResolver {
    static resolveCollision(body1, body2, collision) {
        if (!collision.hasCollision) return;
        body1.wakeUp('collision'); body2.wakeUp('collision');
        this.addCollisionToHistory(body1, body2, collision);
        this.resolvePositionCorrection(body1, body2, collision);

        const r1 = collision.contactPoint.clone().sub(body1.position);
        const r2 = collision.contactPoint.clone().sub(body2.position);

        const v1 = body1.velocity.clone().add(this.crossProduct(body1.angularVelocity, r1));
        const v2 = body2.velocity.clone().add(this.crossProduct(body2.angularVelocity, r2));
        const relativeVelocity = v1.sub(v2);
        const velocityAlongNormal = relativeVelocity.dot(collision.normal);
        if (velocityAlongNormal > 0) return;

        const restitution = this.calculateDynamicRestitution(body1, body2, velocityAlongNormal);
        const impulseScalar = this.calculateImpulseScalar(body1, body2, collision, r1, r2, velocityAlongNormal, restitution);
        const impulse = collision.normal.clone().multiply(impulseScalar);

        if (!body1.isStatic) body1.applyImpulse(impulse.clone().multiply(-1));
        if (!body2.isStatic) body2.applyImpulse(impulse);

        if (!body1.isStatic) {
            const angularImpulse1 = this.crossProduct(r1, impulse.clone().multiply(-1));
            body1.applyAngularImpulse(angularImpulse1);
        }
        if (!body2.isStatic) {
            const angularImpulse2 = this.crossProduct(r2, impulse);
            body2.applyAngularImpulse(angularImpulse2);
        }

        this.applyAdvancedFriction(body1, body2, collision, relativeVelocity, r1, r2);
        this.calculateCollisionEffects(body1, body2, impulseScalar);
    }

    static resolvePositionCorrection(body1, body2, collision) {
        const percent = 0.8;
        const slop = 0.01;
        const correctionMagnitude = Math.max(collision.penetration - slop, 0) * percent;
        const correction = collision.normal.clone().multiply(correctionMagnitude / (body1.inverseMass + body2.inverseMass));
        if (!body1.isStatic) body1.position.sub(correction.clone().multiply(body1.inverseMass));
        if (!body2.isStatic) body2.position.add(correction.clone().multiply(body2.inverseMass));
    }

    static calculateDynamicRestitution(body1, body2, impactVelocity) {
        let restitution = Math.min(body1.restitution, body2.restitution);
        const speedFactor = Math.abs(impactVelocity);
        if (speedFactor > 5.0) restitution *= Math.exp(-(speedFactor - 5.0) * 0.1);
        if (body1.material === 'metal' || body2.material === 'metal') restitution *= 1.2;
        if (body1.material === 'rubber' || body2.material === 'rubber') restitution *= 1.5;
        return Math.max(0, Math.min(1, restitution));
    }

    static calculateImpulseScalar(body1, body2, collision, r1, r2, velocityAlongNormal, restitution) {
        let j = -(1 + restitution) * velocityAlongNormal;
        const r1CrossN = this.crossProduct(r1, collision.normal);
        const r2CrossN = this.crossProduct(r2, collision.normal);
        const rotationalEffect1 = body1.inverseMomentOfInertia * r1CrossN.dot(r1CrossN);
        const rotationalEffect2 = body2.inverseMomentOfInertia * r2CrossN.dot(r2CrossN);
        j /= (body1.inverseMass + body2.inverseMass + rotationalEffect1 + rotationalEffect2);
        return j;
    }

    static applyAdvancedFriction(body1, body2, collision, relativeVelocity, r1, r2) {
        const normalVelocity = collision.normal.clone().multiply(relativeVelocity.dot(collision.normal));
        const tangentialVelocity = relativeVelocity.clone().sub(normalVelocity);
        if (tangentialVelocity.length() < 0.001) return;

        const tangent = tangentialVelocity.clone().normalize();
        const staticFriction = Math.min(body1.staticFriction, body2.staticFriction);
        const dynamicFriction = Math.min(body1.dynamicFriction, body2.dynamicFriction);
        const tangentialSpeed = tangentialVelocity.length();
        const frictionCoeff = tangentialSpeed < 0.1 ? staticFriction : dynamicFriction;

        let frictionImpulse = -tangentialSpeed * frictionCoeff;
        frictionImpulse /= (body1.inverseMass + body2.inverseMass);

        const frictionVector = tangent.multiply(frictionImpulse);
        if (!body1.isStatic) {
            body1.applyImpulse(frictionVector.clone().multiply(-1));
            const frictionTorque1 = this.crossProduct(r1, frictionVector.clone().multiply(-1));
            body1.applyAngularImpulse(frictionTorque1);
        }
        if (!body2.isStatic) {
            body2.applyImpulse(frictionVector);
            const frictionTorque2 = this.crossProduct(r2, frictionVector);
            body2.applyAngularImpulse(frictionTorque2);
        }
    }

    static crossProduct(a,b){ return new VroomVector3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x); }

    static addCollisionToHistory(body1, body2, collision) {
        const collisionData = { timestamp: Date.now(), otherBodyId: body2.id, normal: collision.normal.clone(), penetration: collision.penetration, contactPoint: collision.contactPoint.clone() };
        body1.collisionHistory.push(collisionData);
        if (body1.collisionHistory.length > body1.maxCollisionHistory) body1.collisionHistory.shift();

        const collisionData2 = { timestamp: Date.now(), otherBodyId: body1.id, normal: collision.normal.clone().multiply(-1), penetration: collision.penetration, contactPoint: collision.contactPoint.clone() };
        body2.collisionHistory.push(collisionData2);
        if (body2.collisionHistory.length > body2.maxCollisionHistory) body2.collisionHistory.shift();
    }

    static calculateCollisionEffects(body1, body2, impulseScalar) {
        const collisionEnergy = Math.abs(impulseScalar) * 0.5;
        body1.lastCollisionEnergy = collisionEnergy;
        body2.lastCollisionEnergy = collisionEnergy;
        const heatGenerated = collisionEnergy * 0.001;
        body1.temperature = (body1.temperature || 20) + heatGenerated;
        body2.temperature = (body2.temperature || 20) + heatGenerated;
    }

    static resolveGroundCollision(body, collision) {
        if (!collision.hasCollision) return;
        body.wakeUp('ground_collision');
        body