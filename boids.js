// Adapted from https://p5js.org/examples/simulate-flocking.html

// global variables: scene

//=== vector functions ===
const mag = v => Math.sqrt(v.x*v.x + v.y*v.y);
const magSq = v => v.x*v.x + v.y*v.y;
const createVector = (x, y) => { return { x, y }; };
const random = (min, max) => Math.random() * (max-min) + min;
const dist = (p1, p2) => Math.sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y));
const sub = (v1, v2) => createVector(v1.x - v2.x, v1.y - v2.y);
const mult = (v, a) => createVector(v.x * a, v.y * a);
const div = (v, a) => createVector(v.x / a, v.y / a);
const add = (v1, v2) => createVector(v1.x + v2.x, v1.y + v2.y);
const limit = (v, max) => {
  const mSq = magSq(v);
  let x = v.x;
  let y = v.y;
  if (mSq > max * max) {
    x /= Math.sqrt(mSq) * max;
    y /= Math.sqrt(mSq) * max;
  }
  return createVector(x,y);
};
const heading = v => Math.atan2(v.y, v.x);
const radians = degrees => degrees * Math.PI / 180;
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
let width, height;

function setup(width, height, numBoids) {
//  createCanvas(640, 360);
//  createP("Drag the mouse to generate new boids.");

  width = width;
  height = height;
  flock = new Flock();
  // Add an initial set of boids into the system
  for (let i = 0; i < numBoids; i++) {
    const b = new Boid(width / 2,height / 2, 'boid-'+i);
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

function Boid(x, y, id) {
  this.acceleration = createVector(0, 0);
  this.velocity = createVector(random(-.05, .05), random(-.05, .05));
  this.position = createVector(x, y);
  this.r = 3.0;
  this.maxspeed = 3;    // Maximum speed
  this.maxforce = 0.5; // Maximum steering force (orig: 0.03)
  this.id = id;
  // create DOM node
  this.domNode = document.createElement('a-box');
  this.domNode.setAttribute('position', `${this.position.x} ${this.position.y} 0`);
  this.domNode.setAttribute('height', 0.1);
  this.domNode.setAttribute('width', 0.1);
  this.domNode.setAttribute('depth', 0.1);
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
  sep = mult(sep, 1.5);
  ali = mult(ali, 1.0);
  coh = mult(coh, 1.0);
  // Add the force vectors to acceleration
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
/*
  let theta = heading(this.velocity) + radians(90);
  fill(127);
  stroke(200);
  push();
  translate(this.position.x, this.position.y);
  rotate(theta);
  beginShape();
  vertex(0, -this.r * 2);
  vertex(-this.r, this.r * 2);
  vertex(this.r, this.r * 2);
  endShape(CLOSE);
  pop();
*/
  const node = document.getElementById(this.id);
  const theta = heading(this.velocity) + radians(90);
  node.setAttribute('position', `${this.position.x} ${this.position.y} 0`);
  node.setAttribute('rotation', `0 ${theta*360} 0`);
}

// Wraparound
Boid.prototype.borders = function() {
  if (this.position.x < -this.r)  this.position.x = width + this.r;
  if (this.position.y < -this.r)  this.position.y = height + this.r;
  if (this.position.x > width + this.r) this.position.x = -this.r;
  if (this.position.y > height + this.r) this.position.y = -this.r;
}

// Separation
// Method checks for nearby boids and steers away
Boid.prototype.separate = function(boids) {
  let desiredseparation = 25.0;
  let steer = createVector(0, 0);
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
      street = add(steer, diff);
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
    steer.sub(this.velocity);
    steer = limit(steer, this.maxforce);
  }
  return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
Boid.prototype.align = function(boids) {
  let neighbordist = 50;
  let sum = createVector(0,0);
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
    return createVector(0, 0);
  }
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Boid.prototype.cohesion = function(boids) {
  let neighbordist = 50;
  let sum = createVector(0, 0);   // Start with empty vector to accumulate all locations
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
    return createVector(0, 0);
  }
}
