// Adapted from https://p5js.org/examples/simulate-flocking.html

// global variables: scene


//======================
// util

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



let flock;
let width, height, depth;

function setup(w, h, d, numBoids) {
  width=w;
  height=h;
  depth=d;

  flock = new Flock();
  // Add an initial set of boids into the system
  for (let i = 0; i < numBoids; i++) {
    const b = new Boid(width / 2,height / 2, depth / 2, 'boid-'+i);
    flock.addBoid(b);
  }
}

function draw() {
  flock.run();
}

// Add a new boid into the System
function mouseDragged() {
  flock.addBoid(new Boid(mouseX, mouseY));
}

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Flock object
// Does very little, simply manages the array of all the boids

function Flock() {
  // An array for all the boids
  this.boids = []; // Initialize the array
}

Flock.prototype.run = function() {
  for (let i = 0; i < this.boids.length; i++) {
    this.boids[i].run(this.boids);  // Passing the entire list of boids to each boid individually
  }
}

Flock.prototype.addBoid = function(b) {
  this.boids.push(b);
}

// Boid class

function Boid(x, y, z, id) {
  this.acceleration = new THREE.Vector3(0, 0, 0);
  this.velocity = new THREE.Vector3(random(-.01, .01), random(-.01, .01), random(-.01, .01));
  this.position = new THREE.Vector3(x, y, z);
  this.r = 3.0;
  this.maxspeed = 0.7;    // Maximum speed
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
  scene.appendChild(this.domNode);
}


Boid.prototype.run = function(boids) {
  this.flock(boids);
  this.update();
  this.borders();
  this.render();
}


Boid.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}


// We accumulate a new acceleration each time based on three rules
Boid.prototype.flock = function(boids) {
  let sep = this.separate(boids);   // Separation
  let ali = this.align(boids);      // Alignment
  let coh = this.cohesion(boids);   // Cohesion



  // Arbitrarily weight these forces
  sep.multiplyScalar(0.01);
  ali.multiplyScalar(0.0002);
  coh.multiplyScalar(0.0105);

  // Add the force vectors to acceleration
  this.applyForce(sep);
  this.applyForce(ali);
  this.applyForce(coh);
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


// Wraparound
Boid.prototype.borders = function() {
  if (this.position.x < -this.r)  this.position.x = width + this.r;
  if (this.position.y < -this.r)  this.position.y = height + this.r;
  if (this.position.z < -this.r)  this.position.z = depth + this.r;
  if (this.position.x > width + this.r) this.position.x = -this.r;
  if (this.position.y > height + this.r) this.position.y = -this.r;
  if (this.position.z > depth + this.r) this.position.z = -this.r;
}

// Separation
// Method checks for nearby boids and steers away
Boid.prototype.separate = function(boids) {
  let desiredseparation = 25.0;
  let steer = new THREE.Vector3(0, 0, 0);
  let count = 0;
  // For every boid in the system, check if it's too close
  for (let i = 0; i < boids.length; i++) {
    let d = this.position.distanceTo(boids[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      let diff = this.position
        .clone()
        .sub(boids[i].position)
        .normalize()
        .divideScalar(d);
      steer.add(diff);
      count++;            // Keep track of how many
    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer.divideScalar(count);
  }

  // As long as the vector is greater than 0
  if (steer.length() > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer
      .normalize()
      .multiplyScalar(this.maxspeed)
      .sub(this.velocity)
      .limit(this.maxforce);
  }
  return steer;
}


// Alignment
// For every nearby boid in the system, calculate the average velocity
Boid.prototype.align = function(boids) {
  let neighbordist = 50;
  let sum = new THREE.Vector3(0,0,0);
  let count = 0;
  for (let i = 0; i < boids.length; i++) {
    let d = this.position.distanceTo(boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].velocity);
      count++;
    }
  }
  if (count > 0) {
    return sum
      .divideScalar(count)
      .normalize()
      .multiplyScalar(this.maxspeed)
      .sub(this.velocity)
      .limit(this.maxforce)
  } else {
    return new THREE.Vector3(0,0,0);
  }
}


// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Boid.prototype.cohesion = function(boids) {
  let neighbordist = 5;
  let sum = new THREE.Vector3(0,0,0);   // Start with empty vector to accumulate all locations
  let count = 0;
  for (let i = 0; i < boids.length; i++) {
    let d = this.position.distanceTo(boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].position); // Add location
      count++;
    }
  }
  if (count > 0) {
    sum.divideScalar(count);
    return this.seek(sum);  // Steer towards the location
  } else {
    return new THREE.Vector3(0, 0, 0);
  }
}
