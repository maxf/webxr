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
  this.centre = new THREE.Vector3(cx, cy, cz);
  // Add an initial set of boids into the system
  for (let i = 0; i < numBoids; i++) {
    const b = new Boid(cx, cy, cz, `flock-${id}-boid-${i}`);
    this.addBoid(b);
  }
}

Flock.prototype.draw = function() {
  for (let i = 0; i < this.boids.length; i++) {
    this.boids[i].run(this.boids, this.centre);
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
  this.maxspeed = 2;    // Maximum speed
  this.maxforce = 3; // Maximum steering force (orig: 0.03)
  this.id = id;

  // create DOM node
  this.domNode = document.createElement('a-cone');
  this.domNode.setAttribute('position', `${this.position.x} ${this.position.y} ${this.position.z}`);
  this.domNode.setAttribute('height', 0.2);
  this.domNode.setAttribute('radius-bottom', 0.06);
  this.domNode.setAttribute('radius-top', 0.01);
  this.domNode.setAttribute('color', randomColour());
  this.domNode.setAttribute('id', this.id);
  document.getElementById('scene').appendChild(this.domNode);
}


Boid.prototype.run = function(boids, centre) {
  this.flock(boids, centre);
  this.update();
  this.render();
}


Boid.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}


// We accumulate a new acceleration each time based on three rules

Boid.prototype.flock = function(boids, centre) {
  const forces = this.computeForces(boids, centre);

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

Boid.prototype.computeForces = function(boids, centre) {

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
  let recenter = recenterForce(this.position, centre);


  const cohNeighbordistSq = 25000;
  const aliNeighbordistSq = 25000;
  const sepDesiredSq = 0.4;

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
// Computation Shaders
//============================================================

const bounds = 500;

// Create the original position texture with random coordinates between 0 and 100
function fillPositionTexture( texture ) {
  let theArray = texture.image.data;

  for ( let k = 0, kl = theArray.length; k < kl; k += 4 ) {
    const x = Math.random() * 100 ;
    const y = Math.random() * 100;
    const z = Math.random() * 100;

    theArray[ k + 0 ] = x;
    theArray[ k + 1 ] = y;
    theArray[ k + 2 ] = z;
    theArray[ k + 3 ] = 1;
  }
}


// Create the original velocity texture with random values between -5 and 5
function fillVelocityTexture( texture ) {
  var theArray = texture.image.data;

  for ( var k = 0, kl = theArray.length; k < kl; k += 4 ) {
    var x = Math.random() - 0.5;
    var y = Math.random() - 0.5;
    var z = Math.random() - 0.5;

    theArray[ k + 0 ] = x * 10;
    theArray[ k + 1 ] = y * 10;
    theArray[ k + 2 ] = z * 10;
    theArray[ k + 3 ] = 1;
  }
}


// Initialise the 2 shaders we use for computation (and not rendering)
// arguments:
// - nbBoidsSqRt: the size of the square textures used to do computations
//   one texture for boid position, one for boid velocity.
//   Hence the number of boids is nbBoidsSqRt squared.
// - renderer: the webgl renderer. Not used other than to test for GLSL extensions
function initComputeRenderer(nbBoidsSqRt, webGLRenderer) {
  const computeRenderer = new GPUComputationRenderer( nbBoidsSqRt, nbBoidsSqRt, webGLRenderer );

  // create the position and velocity matrices as textures
  const positionTexture = computeRenderer.createTexture();
  const velocityTexture = computeRenderer.createTexture();
  fillPositionTexture(positionTexture);
  fillVelocityTexture(velocityTexture);

  // add them to be accessible to the 2 shaders code
  const positionShaderCode = document.getElementById('BoidPositionFragmentShader').textContent;
  const velocityShaderCode = document.getElementById('BoidVelocityFragmentShader').textContent;
  positionVariable = computeRenderer.addVariable('PositionTexture', positionShaderCode, positionTexture);
  velocityVariable = computeRenderer.addVariable('VelocityTexture', velocityShaderCode, velocityTexture);
  computeRenderer.setVariableDependencies(velocityVariable, [ positionVariable, velocityVariable ]);
  computeRenderer.setVariableDependencies(positionVariable, [ positionVariable, velocityVariable ]);

  // uniforms
  // (global  variables  whose  values  are  the  same  across  theentire primitive being processed)
  uniformPosition = positionVariable.material.uniforms;
  uniformVelocity = velocityVariable.material.uniforms;
  uniformPosition.clock = { value: 0.0 };
  uniformPosition.del_change = { value: 0.0 };

  uniformVelocity.clock = { value: 1.0 };
  uniformVelocity.del_change = { value: 0.0 };
  uniformVelocity.testing = { value: 1.0 };
  uniformVelocity.seperation_distance = { value: 1.0 };
  uniformVelocity.alignment_distance = { value: 1.0 };
  uniformVelocity.cohesion_distance = { value: 1.0 };
  uniformVelocity.freedom_distance = { value: 1.0 };
  uniformVelocity.predator = { value: new THREE.Vector3() };

  // variables
  velocityVariable.material.defines.bounds = bounds.toFixed( 2 );
  velocityVariable.wrapS = THREE.RepeatWrapping;
  velocityVariable.wrapT = THREE.RepeatWrapping;

  positionVariable.wrapS = THREE.RepeatWrapping;
  positionVariable.wrapT = THREE.RepeatWrapping;

  var error = computeRenderer.init();
  if ( error !== null ) {
    console.error( error );
  }
  return computeRenderer;
}

//===============================
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

  multiple: true,

  init: function () {
    const webGLRenderer = document.querySelector('a-scene').renderer;

//    this.computeRenderer = initComputeRenderer(128, webGLRenderer);

    this.data.flock = new Flock(
      this.data.width,
      this.data.height,
      this.data.depth,
      this.data.numBoids,
      this.data.cx,
      this.data.cy,
      this.data.cz,
      this.id
    );
  },

  tick: function (time, timeDelta) {

    const r = x => (Math.round(x * 100) / 100).toFixed(2);

    this.data.flock.draw();
    const pos = document.getElementById('left-hand').getAttribute('position');
    const sphere = document.getElementById('pointer');

    const nX = 20*pos.x;
    const nY = 10*pos.y-2;
    const nZ = 20*pos.z;

    sphere.setAttribute('position', `${nX} ${nY} ${nZ}`);
//    message(`${r(nX)},${r(nY)},${r(nZ)}`);;
    this.data.flock.centre.x = nX;
    this.data.flock.centre.y = nY;
    this.data.flock.centre.z = nZ;

//    this.computeRenderer.compute();

//    uniform_bird.PositionTexture.value = gpu_allocation.getCurrentRenderTarget( position_variable ).texture;
//    uniform_bird.VeloctiyTexture.value = gpu_allocation.getCurrentRenderTarget( velocity_variable ).texture;

  }

});
