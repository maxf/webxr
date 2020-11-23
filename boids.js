// Adapted from https://p5js.org/examples/simulate-flocking.html

// global variables: scene

//=== vector functions ===
const mag = v => Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
const magSq = v => v.x*v.x + v.y*v.y + v.z*v.z;
const createVector = (x, y, z) => { return { x, y, z }; };
const random = (min, max) => Math.random() * (max-min) + min;
const dist = (p1, p2) => Math.sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) + (p2.z-p1.z)*(p2.z-p1.z));
const sub = (v1, v2) => createVector(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
const mult = (v, a) => createVector(v.x * a, v.y * a, v.z*a);
const div = (v, a) => createVector(v.x / a, v.y / a, v.z/a);
const add = (v1, v2) => createVector(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
const limit = (v, max) => {
  const mSq = magSq(v);
  if (mSq > max * max) {
    const f = Math.sqrt(mSq) * max;
    return createVector(v.x/f, v.y/f, v.z/f);
  } else {
    return createVector(v.x,v.y,v.z);
  }
};
//const heading = v => Math.atan2(v.y, v.x);
const radians = degrees => degrees * Math.PI / 180;
const degrees = radians => radians * 180 / Math.PI;
const normalize = v => limit(v, 1);

//======================
// util

const paletteColour = i => {
  const palette = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf'];
  return palette[i%palette.length];
};

const randomColour = () => paletteColour(Math.round(Math.random()*1000));



//======================



let flock;
let width, height, depth;

function setup(w, h, d, numBoids) {
  console.log(24, numBoids);
  width=w;
  height=h;
  depth=d;

  flock = new Flock();
  // Add an initial set of boids into the system
  for (let i = 0; i < numBoids; i++) {
//    const b = new Boid(0,height / 2, depth/2, 'boid-'+i);
//    const b = new Boid(height / 2, depth/2, 0, 'boid-'+i);
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

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Boid class
// Methods for Separation, Cohesion, Alignment added

function Boid(x, y, z, id) {
  this.acceleration = createVector(0, 0, 0);
//  this.velocity = createVector(0, random(-.01, .01), random(-.01, .01));
//  this.velocity = createVector(random(-.01, .01), random(-.01, .01), 0);
  this.velocity = createVector(random(-.01, .01), random(-.01, .01), random(-.01, .01));
  this.position = createVector(x, y, z);
  this.r = 3.0;
  this.maxspeed = 3;    // Maximum speed
  this.maxforce = 0.5; // Maximum steering force (orig: 0.03)
  this.id = id;
  // create DOM node
  this.domNode = document.createElement('a-cone');
  this.domNode.setAttribute('position', `${this.position.x} ${this.position.y} ${this.position.z}`);
  this.domNode.setAttribute('height', 0.1);
  this.domNode.setAttribute('radius-bottom', 0.03);
  this.domNode.setAttribute('radius-top', 0);
  this.domNode.setAttribute('color', randomColour()); //'#4cd93c');
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
  this.acceleration = add(this.acceleration, force);
}

// We accumulate a new acceleration each time based on three rules
Boid.prototype.flock = function(boids) {
  let sep = this.separate(boids);   // Separation
  let ali = this.align(boids);      // Alignment
  let coh = this.cohesion(boids);   // Cohesion
  // Arbitrarily weight these forces
  sep = mult(sep, 0.01);
  ali = mult(ali, 0.02);
  coh = mult(coh, 0.01);
  // sep = mult(sep, 1.5);
  // ali = mult(ali, 1.0);
  // coh = mult(coh, 1.0);
  // Add the force vectors to acceleration
//  console.log(sep, ali, coh);
  this.applyForce(sep);
  this.applyForce(ali);
  this.applyForce(coh);
}

// Method to update location
Boid.prototype.update = function() {
  // Update velocity
  this.velocity = add(this.velocity, this.acceleration);
  // Limit speed
  this.velocity = limit(this.velocity, this.maxspeed);
  this.position = add(this.position, this.velocity);

  // Reset accelertion to 0 each cycle
  this.acceleration = mult(this.acceleration, 0);
}

// A method that calculates and applies a steering force towards a target
// STEER = DESIRED MINUS VELOCITY
Boid.prototype.seek = function(target) {
  let desired = sub(target,this.position);  // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired = normalize(desired);
  desired = mult(desired, this.maxspeed);
  // Steering = Desired minus Velocity
  let steer = sub(desired,this.velocity);
  let steer2 = limit(steer, this.maxforce);  // Limit to maximum steering force
  return steer2;
}

Boid.prototype.render = function() {
  // Draw a triangle rotated in the direction of velocity
  const node = document.getElementById(this.id);
  const [ vx, vy, vz ] = [ this.velocity.x, this.velocity.y, this.velocity.z ];
  node.setAttribute('position', `${this.position.x} ${this.position.y} ${this.position.z}`);

//  const thetaX = Math.atan2(vz, vy);
//  const thetaZ = Math.atan2(vy, vx);

  const thetaX = Math.atan2(vz, vy);
  const thetaZ = -Math.atan2(vx, Math.sqrt(vy*vy + vz*vz));

//  node.object3D.rotation.set(thetaX, 0, thetaZ);

//  const eulerAngles = new THREE.Euler(thetaX, 0, thetaZ);
//  node.object3D.rotation = eulerAngles;


//  node.setAttribute('rotation', `${thetaX*180/Math.PI} 0 0`);
//  node.setAttribute('rotation', `0 0 ${thetaZ*180/Math.PI - 90}`);
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
  let steer = createVector(0, 0, 0);
  let count = 0;
  // For every boid in the system, check if it's too close
  for (let i = 0; i < boids.length; i++) {
    let d = dist(this.position,boids[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      let diff = sub(this.position, boids[i].position);
      diff = normalize(diff);
      diff = div(diff, d);        // Weight by distance
      steer = add(steer, diff);
      count++;            // Keep track of how many
    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer = div(steer, count);
  }

  // As long as the vector is greater than 0
  if (mag(steer) > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer = normalize(steer);
    steer = mult(steer, this.maxspeed);
    steer = sub(steer, this.velocity);
    steer = limit(steer, this.maxforce);
  }
  return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
Boid.prototype.align = function(boids) {
  let neighbordist = 50;
  let sum = createVector(0,0,0);
  let count = 0;
  for (let i = 0; i < boids.length; i++) {
    let d = dist(this.position,boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum = add(sum, boids[i].velocity);
      count++;
    }
  }
  if (count > 0) {
    let sum2 = div(sum, count);
    let sum3 = normalize(sum2);
    let sum4 = mult(sum3, this.maxspeed);
    let steer = sub(sum4, this.velocity);
    let steer2 = limit(steer, this.maxforce);
    return steer2;
  } else {
    return createVector(0,0,0);
  }
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Boid.prototype.cohesion = function(boids) {
  let neighbordist = 50;
  let sum = createVector(0,0,0);   // Start with empty vector to accumulate all locations
  let count = 0;
  for (let i = 0; i < boids.length; i++) {
    let d = dist(this.position,boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum = add(sum, boids[i].position); // Add location
      count++;
    }
  }
  if (count > 0) {
    sum = div(sum, count);
    return this.seek(sum);  // Steer towards the location
  } else {
    return createVector(0, 0, 0);
  }
}
