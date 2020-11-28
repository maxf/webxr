// Adapted from https://p5js.org/examples/simulate-flocking.html


//======================
// util

const rnd = (min, max) => Math.random()*(max-min) + min;

const limit = (v, max) => {
  const mSq = v.lengthSq();
  if (mSq > max * max) {
    const f = Math.sqrt(mSq) * max;
    return new THREE.Vec3(v.x/f, v.y/f, v.z/f);
  } else {
    return new THREE.Vec3(v.x,v.y,v.z);
  }
};

THREE.Vector3.prototype.limit = function(max) {
  const mSq = this.lengthSq();
  if (mSq > max * max) {
    this.divideScalar(Math.sqrt(mSq) * max);
  }
  return this;
};

const radians = degrees => degrees * Math.PI / 180;
const degrees = radians => radians * 180 / Math.PI;

const random = (min, max) => Math.random() * (max-min) + min;

const paletteColour = i => {
  const palette = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf'];
  return palette[i%palette.length];
};

const randomColour = () => paletteColour(Math.round(Math.random()*1000));


//======================

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Flock object
// Does very little, simply manages the array of all the boids

function Flock(w, h, d, numBoids, cx, cy, cz, id) {
  this.boids = [];
  this.width=w;
  this.height=h;
  this.depth=d;
  this.numBoids = numBoids;
  this.id = id;
  // Add an initial set of boids into the system
  for (let i = 0; i < numBoids; i++) {
    const b = new Boid(cx, cy, cz, cx, cy, cz, `flock-${id}-boid-${i}`);
    this.addBoid(b);
  }
}

Flock.prototype.draw = function() {
  for (let i = 0; i < this.boids.length; i++) {
    this.boids[i].run(this.boids);  // Passing the entire list of boids to each boid individually
  }
}


Flock.prototype.addBoid = function(b) {
  this.boids.push(b);
}


// Boid class

function Boid(x, y, z, cx, cy, cz, id) {
  this.acceleration = new THREE.Vector3(0, 0, 0);
  this.velocity = new THREE.Vector3(random(-.01, .01), random(-.01, .01), random(-.01, .01));
  this.position = new THREE.Vector3(x, y, z);
  this.centre = new THREE.Vector3(cx, cy, cz);
  this.maxspeed = 3;    // Maximum speed
  this.maxforce = 0.5; // Maximum steering force (orig: 0.03)
  this.id = id;

  // create DOM node
  this.domNode = document.createElement('a-cone');
  this.domNode.setAttribute('position', `${this.position.x} ${this.position.y} ${this.position.z}`);
  this.domNode.setAttribute('height', 0.1);
  this.domNode.setAttribute('radius-bottom', 0.03);
  this.domNode.setAttribute('radius-top', 0);
  this.domNode.setAttribute('color', randomColour());
  this.domNode.setAttribute('id', this.id);
  document.getElementById('scene').appendChild(this.domNode);
}


Boid.prototype.run = function(boids) {
  this.flock(boids);
  this.update();
  this.render();
}


Boid.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}


// We accumulate a new acceleration each time based on three rules

Boid.prototype.flock = function(boids) {
  const forces = this.computeForces(boids);

  this.applyForce(forces.separation.multiplyScalar(0.0107));
  this.applyForce(forces.alignment.multiplyScalar(0.0002));
  this.applyForce(forces.cohesion.multiplyScalar(0.0105));
  this.applyForce(forces.recenter.multiplyScalar(0.0005));
}


// Method to update location
Boid.prototype.update = function() {
  // Update velocity
  this.velocity.add(this.acceleration);
  // Limit speed
  this.velocity.limit(this.maxspeed);
  this.position.add(this.velocity);

  // Reset accelertion to 0 each cycle
  this.acceleration.set(0, 0, 0);
}


// A method that calculates and applies a steering force towards a target
Boid.prototype.seek = function(target) {
  return target
    .clone()
    .sub(this.position) // A vector pointing from the location to the target
    .normalize()
    .multiplyScalar(this.maxspeed) // Normalize desired and scale to maximum speed
    .sub(this.velocity) // Steering = Desired minus Velocity
    .limit(this.maxforce); // Limit to maximum steering force
};


Boid.prototype.render = function() {
  // Draw a triangle rotated in the direction of velocity
  const node = document.getElementById(this.id);
  node.setAttribute('position', `${this.position.x} ${this.position.y} ${this.position.z}`);

  const [ vx, vy, vz ] = [ this.velocity.x, this.velocity.y, this.velocity.z ];
  const thetaX = Math.atan2(vz, vy);
  const thetaZ = -Math.atan2(vx, Math.sqrt(vy*vy + vz*vz));
  node.setAttribute('rotation', `${degrees(thetaX)} 0 ${degrees(thetaZ)}`);
}


const recenterForce = function(position, centre) {
  return centre.clone().sub(position);
};

//----------------------------------------------------------------------------

Boid.prototype.computeForces = function(boids) {

  // Cohesion
  // For the average location (i.e. center) of all nearby boids,
  // calculate steering vector towards that location
  let cohesion = new THREE.Vector3(0,0,0);

  // Separation
  // Method checks for nearby boids and steers away
  let separation = new THREE.Vector3(0,0,0);

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  let alignment = new THREE.Vector3(0,0,0);

  // Recenter
  // Force to move boid back to center
  let recenter = recenterForce(this.position, this.centre);


  const cohNeighbordistSq = 25;
  const aliNeighbordistSq = 250;
  const sepDesiredSq = 225;

  let cohCount = 0, aliCount = 0, sepCount = 0;

  for (let i = 0; i < boids.length; i++) {
    const d = this.position.distanceToSquared(boids[i].position);
    if (d > 0) {
      if (d < cohNeighbordistSq) {
        cohesion.add(boids[i].position);
        cohCount++;
      }
      if (d < aliNeighbordistSq) {
        alignment.add(boids[i].velocity);
        aliCount++;
      }
      if (d < sepDesiredSq) {
        const diff = this.position
          .clone()
          .sub(boids[i].position)
          .normalize()
          .divideScalar(Math.sqrt(d));
        separation.add(diff);
        sepCount++;
      }
    }
  }

  if (cohCount > 0) {
    cohesion.divideScalar(cohCount);
    cohesion = this.seek(cohesion);
  }

  if (aliCount > 0) {
    alignment
      .divideScalar(aliCount)
      .normalize()
      .multiplyScalar(this.maxspeed)
      .sub(this.velocity)
      .limit(this.maxforce)
  }

  if (separation.x !== 0 && separation.y !== 0 && separation.z !== 0) {
    if (sepCount > 0) {
      separation.divideScalar(sepCount);
    }
    separation
      .normalize()
      .multiplyScalar(this.maxspeed)
      .sub(this.velocity)
      .limit(this.maxforce);
  }

  return { cohesion, alignment, separation, recenter };
}



//============================================================

AFRAME.registerComponent('boids', {
  schema: {
    width: { type: 'number', default: 5 },
    height: { type: 'number', default: 5 },
    depth: { type: 'number', default: 5 },
    numBoids: { type: 'number', default: 500 },
    cx: { type: 'number', default: 0 },
    cy: { type: 'number', default: 0 },
    cz: { type: 'number', default: 0 }
  },

  init: function () {
    this.data.flock = new Flock(
      this.data.width,
      this.data.height,
      this.data.depth,
      this.data.numBoids,
      this.data.cx,
      this.data.cy,
      this.data.cz,
      `f${new Date().getTime()}`
    );
  },

  tick: function (time, timeDelta) {
    this.data.flock.draw();
  }

});
