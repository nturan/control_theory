import * as THREE from 'https://unpkg.com/three/build/three.module.js';
import {PhysicsBody} from "./physics_engine/physics_engine.js";

////////////////////////////////////////////////////////////////////////////////
class simObj{
  constructor(mass, position, engine, scale, mesh, pidMatrix){
    this.mass = mass;
    this.position = position;
    this.engine = engine; //as force
    this.scale = scale;
    this.mesh = mesh;
    this.pidMatrix = pidMatrix;
    this.velocity = {x: 0, y: 0, z: 0};
    this.physics_body = new PhysicsBody(mass, position, this.velocity, this.calcInertiaTensor.bind(this));
    
    this.thrust = {LF: 0, RF: 0, LB: 0, RB: 0};
    this.engMassRatio = 0.25;
    this.landed = false;
    this.time = 0;
    this.counter = 0;
    this.counter2 = 0;
    this.setPoint = [0, 0, 0, 0];
    this.objective = {mode: null, start: null, duration: null};
    this.tl = null;
    this.telemetry();
    this.display = "Idle";
    this.calc = [{counter: 0, limit: 30,
                  event: function(actor){actor.masterPID();}},
                 {counter: 0, limit: 17, 
                  event: function(actor){actor.secondPID();}},
                 {counter: 0,  limit: 0,  
                  event: function(actor){actor.thirdPID();}}];
    this.gas = 0;
    this.pitch = 0;
    this.roll = 0;
    this.yaw = 0;
  }

  ApplyThrust(){
    this.physics_body.ApplyForce(this.calcForce());
    this.physics_body.ApplyTorque(this.calcTorque());
  }
  
  calcForce(){
    let totalThrust = new THREE.Vector3(0, ( this.thrust.LF
                                            +this.thrust.RF
                                            +this.thrust.LB
                                            +this.thrust.RB)
                                           *this.engine*this.mass, 0);
    totalThrust.applyQuaternion(this.mesh.quaternion);
    
//    let F = math.add([this.force.x, 
//                      this.force.y, 
//                      this.force.z], totalThrust.toArray());

    return totalThrust;
  }
  
  calcTorque(){
    let posLF = new THREE.Vector3(  1*this.scale, 0,  1*this.scale);
    let posRF = new THREE.Vector3( -1*this.scale, 0,  1*this.scale);
    let posLB = new THREE.Vector3(  1*this.scale, 0, -1*this.scale);
    let posRB = new THREE.Vector3( -1*this.scale, 0, -1*this.scale);
    
    let thrustLF = new THREE.Vector3(0,this.thrust.LF*this.engine*this.mass,0);
    let thrustRF = new THREE.Vector3(0,this.thrust.RF*this.engine*this.mass,0);
    let thrustLB = new THREE.Vector3(0,this.thrust.LB*this.engine*this.mass,0);
    let thrustRB = new THREE.Vector3(0,this.thrust.RB*this.engine*this.mass,0);

    
    let torqueLF = new THREE.Vector3();
    let torqueRF = new THREE.Vector3();
    let torqueLB = new THREE.Vector3();
    let torqueRB = new THREE.Vector3();
    torqueLF.crossVectors( posLF.applyQuaternion(this.mesh.quaternion), 
                           thrustLF.applyQuaternion(this.mesh.quaternion));
    torqueRF.crossVectors( posRF.applyQuaternion(this.mesh.quaternion), 
                           thrustRF.applyQuaternion(this.mesh.quaternion));
    torqueLB.crossVectors( posLB.applyQuaternion(this.mesh.quaternion), 
                           thrustLB.applyQuaternion(this.mesh.quaternion));
    torqueRB.crossVectors( posRB.applyQuaternion(this.mesh.quaternion), 
                           thrustRB.applyQuaternion(this.mesh.quaternion));
    
    
    
    
    //Artificial Yaw Torque
    let semR = this.scale*this.engine*this.mass*this.engMassRatio;
    let torqueYawLF = new THREE.Vector3(0,-this.thrust.LF*semR, 0);
    let torqueYawRB = new THREE.Vector3(0,-this.thrust.RB*semR, 0);
    let torqueYawRF = new THREE.Vector3(0, this.thrust.RF*semR, 0);
    let torqueYawLB = new THREE.Vector3(0, this.thrust.LB*semR, 0);
    
    torqueYawLF.applyQuaternion(this.mesh.quaternion);
    torqueYawRF.applyQuaternion(this.mesh.quaternion);
    torqueYawLB.applyQuaternion(this.mesh.quaternion);
    torqueYawRB.applyQuaternion(this.mesh.quaternion);
    let torque = math.add(torqueLF.toArray(), 
    torqueRF.toArray(), 
    torqueLB.toArray(), 
    torqueRB.toArray(),
    torqueYawLF.toArray(),
    torqueYawRF.toArray(),
    torqueYawLB.toArray(),
    torqueYawRB.toArray());
    return {x: torque[0], y: torque[1], z: torque[2]};
  }
  
  calcInertiaTensor(){
    let posLF = [[ 1*this.scale], [0], [ 1*this.scale]]; // column vector
    let posRF = [[-1*this.scale], [0], [ 1*this.scale]];
    let posLB = [[ 1*this.scale], [0], [-1*this.scale]];
    let posRB = [[-1*this.scale], [0], [-1*this.scale]];
    
    let tPosLF = [[ 1*this.scale, 0,  1*this.scale]] // row vector
    let tPosRF = [[-1*this.scale, 0,  1*this.scale]];
    let tPosLB = [[ 1*this.scale, 0, -1*this.scale]];
    let tPosRB = [[-1*this.scale, 0, -1*this.scale]];
    
    let tPosLFxposLF = math.multiply(tPosLF, posLF)[0][0]
    let posLFxtPosLF = math.multiply(posLF, tPosLF)
    let inerLF = math.subtract(math.multiply(tPosLFxposLF, 
                                             this.mass/4, 
                                             math.identity(3)), 
                               math.multiply(posLFxtPosLF, 
                                             this.mass/4));
    let tPosRFxposRF = math.multiply(tPosRF, posRF)[0][0]
    let posRFxtPosRF = math.multiply(posRF, tPosRF)
    let inerRF = math.subtract(math.multiply(tPosRFxposRF, 
                                             this.mass/4, 
                                             math.identity(3)), 
                               math.multiply(posRFxtPosRF, 
                                             this.mass/4));
    let tPosLBxposLB = math.multiply(tPosLB, posLB)[0][0]
    let posLBxtPosLB = math.multiply(posLB, tPosLB)
    let inerLB = math.subtract(math.multiply(tPosLBxposLB, 
                                             this.mass/4, 
                                             math.identity(3)), 
                               math.multiply(posLBxtPosLB, 
                                             this.mass/4));
    let tPosRBxposRB = math.multiply(tPosRB, posRB)[0][0]
    let posRBxtPosRB = math.multiply(posRB, tPosRB)
    let inerRB = math.subtract(math.multiply(tPosRBxposRB, 
                                             this.mass/4, 
                                             math.identity(3)), 
                               math.multiply(posRBxtPosRB, 
                                             this.mass/4));
    
    let inerTensor0 = math.add(inerLF, inerRF, inerLB, inerRB);
    let R = this.quaternionToMatrix(this.mesh.quaternion);
    return math.multiply(R, inerTensor0, math.transpose(R));
  }
  
  quaternionToMatrix(q){
    return [[1-2*q.y*q.y-2*q.z*q.z, 2*q.x*q.y-2*q.w*q.z, 2*q.x*q.z+2*q.w*q.y],
            [2*q.x*q.y+2*q.w*q.z, 1-2*q.x*q.x-2*q.z*q.z, 2*q.y*q.z-2*q.w*q.x],
            [2*q.x*q.z-2*q.w*q.y, 2*q.y*q.z+2*q.w*q.x, 1-2*q.x*q.x-2*q.y*q.y]];
  }
  
  

  flightController(step){
    this.step = step;
    //calculations
    if(this.calc[0].counter < this.calc[0].limit){
      this.calc[0].counter += 1;
    }else{
      this.calc[0].event(this);
      this.calc[0].counter = 0;
    }
    if(this.calc[1].counter < this.calc[1].limit){
      this.calc[1].counter += 1;
    }else{
      this.calc[1].event(this);
      this.calc[1].counter = 0;
    }
    if(this.calc[2].counter < this.calc[2].limit){
      this.calc[2].counter += 1;
    }else{
      this.calc[2].event(this);
      this.calc[2].counter = 0;
    }
    
    //gas = 0;
    this.thrust.LF = this.gas-this.pitch+this.roll-this.yaw;
    this.thrust.RF = this.gas-this.pitch-this.roll+this.yaw;
    this.thrust.LB = this.gas+this.pitch+this.roll+this.yaw;
    this.thrust.RB = this.gas+this.pitch-this.roll-this.yaw;
  }

  limitControl(control, limit){
    limit = Math.abs(limit);
    if (control > limit){ 
      control = limit;
    }else if(control < -limit){
      control =-limit;
    }
    return control;
  }



  telemetry(){
    this.tl = {px: this.physics_body.position.x,
               py: this.physics_body.position.y,
               pz: this.physics_body.position.z,
               vx: this.physics_body.velocity.x,
               vy: this.physics_body.velocity.y,
               vz: this.physics_body.velocity.z,
               wx: this.physics_body.w.x,
               wy: this.physics_body.w.y,
               wz: this.physics_body.w.z,
               rx: this.physics_body.euler.x,
               ry: this.physics_body.euler.y,
               rz: this.physics_body.euler.z};
  }
  

  printTelemetry(info, dec){
    console.log(this.display);
    for (var i = 0; i < info.length; i++){
      if(info[i] == "position"){
        console.log("pos: " + math.round(this.tl.px, dec) + ", "
                            + math.round(this.tl.py, dec) + ", "
                            + math.round(this.tl.pz, dec) + ". ");
      }
      if(info[i] == "velocity"){
        console.log("vel: " + math.round(this.tl.vx, dec) + ", "
                            + math.round(this.tl.vy, dec) + ", "
                            + math.round(this.tl.vz, dec) + ". ");
      }
      if(info[i] == "rotation"){
        console.log("rot: " + math.round(this.tl.rx, dec) + ", "
                            + math.round(this.tl.ry, dec) + ", "
                            + math.round(this.tl.rz, dec) + ". ");
      }
      if(info[i] == "angular velocity"){
        console.log("ang: " + math.round(this.tl.wx, dec) + ", "
                            + math.round(this.tl.wy, dec) + ", "
                            + math.round(this.tl.wz, dec) + ". ");
      }
    }
  }

  masterPID(){
    this.pidMatrix.gas.master = simPhys.loopPID(this.setPoint[0],
                                                this.tl.py, 0,
                                                this.pidMatrix.gas.master.er,
                                                this.step,
                                           {kP: 1.0E-0, kI: 0, kD: 1.0E-4});
    

    // update speed for horizontal movement every half second. via PID.
    this.pidMatrix.pitch.master = simPhys.loopPID(this.setPoint[1],
                                          this.tl.pz, 0, 
                                          this.pidMatrix.pitch.master.er,
                                          this.step,
                                          {kP: 1.0E-0, kI: 0, kD: 3.0E-3});

    this.pidMatrix.roll.master = simPhys.loopPID(-this.setPoint[2],
                                         -this.tl.px, 0, 
                                          this.pidMatrix.roll.master.er,
                                          this.step,
                                          {kP: 1.0E-0, kI: 0, kD: 3.0E-3});

    let vel = new THREE.Vector3(this.pidMatrix.roll.master.c,
                                this.pidMatrix.gas.master.c,
                                this.pidMatrix.pitch.master.c);
    vel.setLength(3);


    this.pidMatrix.gas.master.c = 
      this.limitControl(this.pidMatrix.gas.master.c, vel.y);
    this.pidMatrix.pitch.master.c = 
      this.limitControl(this.pidMatrix.pitch.master.c, vel.z);
    this.pidMatrix.roll.master.c = 
      this.limitControl(this.pidMatrix.roll.master.c, vel.x);
  }


  secondPID(){
    
    this.pidMatrix.pitch.second = simPhys.loopPID(
                                   this.pidMatrix.pitch.master.c, 
                                   this.tl.vz, 0, 
                               this.pidMatrix.pitch.second.er, 
                                   this.step,
                                 {kP: 5.0E-2, kI: 0, kD: 1.0E-2});



    this.pidMatrix.roll.second = simPhys.loopPID(
                                   this.pidMatrix.roll.master.c, 
                                   -this.tl.vx, 0, 
                               this.pidMatrix.roll.second.er, 
                                   this.step,
                                 {kP: 5.0E-2, kI: 0, kD: 1.0E-2});

  }
  
  thirdPID(){
    this.gas = 9.81/4/this.engine;   
    this.pidMatrix.gas.third = simPhys.loopPID(
                                 this.pidMatrix.gas.master.c, 
                                 this.tl.vy, 0, 
                                 this.pidMatrix.gas.third.er,
                                 this.step,
                                 {kP: 5.0E-1, kI: 0, kD: 0});
   
    this.gas += this.pidMatrix.gas.third.c;
    this.gas = this.limitControl(this.gas, 0.7);

    this.yaw = 0;
    this.pidMatrix.yaw.third = simPhys.loopPID(this.setPoint[3], 
                                 this.tl.ry, 0, 
                                 this.pidMatrix.yaw.third.er, 
                                 this.step,
                                 {kP: 1.0E-1, kI: 0, kD: 5.0E-1});
    this.yaw = this.pidMatrix.yaw.third.c;
    this.yaw = this.limitControl(this.yaw, 0.3);

    this.pidMatrix.pitch.third = simPhys.loopPID(
                               this.pidMatrix.pitch.second.c, 
                                   this.tl.wx, 0, 
                               this.pidMatrix.pitch.third.er, 
                                   this.step,
                                 {kP: 1.3E-1, kI: 0, kD: 1.0E-6});

    this.pitch = this.pidMatrix.pitch.third.c;
    this.pitch = this.limitControl(this.pitch, 0.3);

    this.pidMatrix.roll.third = simPhys.loopPID(
                               this.pidMatrix.roll.second.c, 
                                   this.tl.wz, 0, 
                               this.pidMatrix.roll.third.er, 
                                   this.step,
                                 {kP: 1.3E-1, kI: 0, kD: 1.0E-6});

    this.roll = this.pidMatrix.roll.third.c;
    this.roll = this.limitControl(this.roll, 0.3);
  }
}

export {simObj};