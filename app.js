import * as THREE from "https://cdn.skypack.dev/three";
import { OrbitControls } from 'https://cdn.skypack.dev/three/examples/jsm/controls/OrbitControls.js';
import { simObj } from "./simObj.js"
import {Integrator} from "./physics_engine/physics_engine.js";


angular.module('controlTheoryApp', []).controller('MainController', function ($scope){
  
    let main = this;
    main.pauseSim = false;
    let setAlt = 1.0;
    let setYaw = 0.0;
    main.nwind = 0;
    main.ewind = 0;
    main.density = 1.0;
    //Time Control Variables
    let startCounter = new Date().getTime();
    let frmTime = startCounter;
    let phsCycle = 1/60;
    let gravity = 9.81;


    
    let scene, camera, renderer, controls;


         
    // actor parameters
    let engineInGravity = 0.5*gravity;
    let actor = new simObj( 1, {x: 0, y: 5, z: 0}, engineInGravity, 0.1,
                       buildActorMesh({x:0.1, y:0.1, z:0.1}), buildPIDMatrix());
    var axesHelper = new THREE.AxesHelper( 1 );
    var raycaster = new THREE.Raycaster();
    var mouse = new THREE.Vector2();

    var destArrow = new THREE.Mesh(new THREE.TorusGeometry(0.1,0.01,5,10), 
                                   new THREE.MeshLambertMaterial(
                                   {emissive: 0xed07ce,
                                    color: 0xed07ce}));

  
    destArrow.rotation.x = Math.PI/2;
    let ground_texture = new THREE.TextureLoader().load("resources/ground_texture.jpg");
    ground_texture.wrapS = THREE.RepeatWrapping;
    ground_texture.wrapT = THREE.RepeatWrapping;
    ground_texture.repeat.set( 50, 50 );
    let ground = new THREE.Mesh(
      new THREE.PlaneGeometry(100, 100, ),
      new THREE.MeshPhongMaterial({map: ground_texture, side: THREE.DoubleSide})
    );
    ground.rotateX(math.PI/2);
    ground.translateZ(1);



    

    main.toggleHelp = function(){
      if(main.help){
        document.getElementById("help").style.display = "block";
      }else{
        document.getElementById("help").style.display = "none";
      }
    };


    function onMouseMove ( event ) {
 
      // calculate mouse position in normalized device coordinates
      // (-1 to +1) for both components


      mouse.x = (event.clientX / window.innerWidth ) * 2 - 1;
      mouse.y = - (event.clientY / window.innerHeight ) * 2 + 1;
    }


    function onMouseDown(event){
      switch (event.button){
        case 2:
          let click = pickVector3FromScene(ground, 
                                           mouse, camera);
          actor.setPoint = [click.y+setAlt, 
                                click.z,
                                click.x, 
                                setYaw*Math.PI/90];
           destArrow.position.x = click.x;
           destArrow.position.y = click.y;
           destArrow.position.z = click.z;
           actor.pidMatrix.pitch.master.er = 0;
           actor.pidMatrix.pitch.second.er = 0;
           actor.pidMatrix.pitch.third.er = 0;
           actor.pidMatrix.roll.master.er = 0;
           actor.pidMatrix.roll.second.er = 0;
           actor.pidMatrix.roll.third.er = 0;
           break;
      }
    }

    function pickVector3FromScene(plane, mouse, camera) {
      // update the picking ray with the camera and mouse position
         
         raycaster.setFromCamera(mouse, camera);
      // calculate objects intersecting the picking ray
         let intersect = raycaster.intersectObject(plane);
         return intersect[0].point;
    }  
          
    
    window.addEventListener('resize', onWindowResize, false);
    document.addEventListener('keydown', onDocumentKeyDown, false);
    document.addEventListener('keyup', onDocumentKeyUp, false);
    window.addEventListener('mousemove', onMouseMove, false);
    window.addEventListener('mousedown', onMouseDown, false);

    init();
    animate();
    function init(){
      scene = new THREE.Scene();
      camera = new THREE.PerspectiveCamera( 40, window.innerWidth / window.innerHeight, 0.01, 1000 );
      let amLight = new THREE.AmbientLight(0x333333);
      let sunLight = new THREE.DirectionalLight();
      sunLight.position.x = -3;
      scene.add(amLight);
      scene.add(sunLight);
      renderer = new THREE.WebGLRenderer({antialias: true});
      renderer.setSize(window.innerWidth, window.innerHeight);
      document.body.appendChild(renderer.domElement);
      controls = new OrbitControls(camera, renderer.domElement);

      camera.position.set(-3, 3, -3);
      controls.maxDistance = 10;
//      controls.enableKeys = true;
      controls.enableDamping = true;
      controls.update();
      scene.add( axesHelper );        

      actor.mesh.position.x = actor.position.x;
      actor.mesh.position.y = actor.position.y;
      actor.mesh.position.z = actor.position.z;
      scene.add(actor.mesh);
      actor.setPoint = [2+setAlt, 
                            0.0,
                            0.0, 
                            setYaw*Math.PI/90];
      scene.add( destArrow );
      scene.add( ground );
      }


////////////////////////////////////////////////////////////////////////////////
          
    function physTick(){
      actor.physics_body.ApplyForce({x: 0, y: -gravity*actor.mass, z:0});      
      //air resistance
      actor.physics_body.ApplyForce(simPhys.airResistance(actor.physics_body.velocity, main.density));
      //wind
      actor.physics_body.ApplyForce({x: main.nwind*main.density, 
                                     y: 0, 
                                     z:-main.ewind*main.density});
      actor.flightController(phsCycle);
      actor.ApplyThrust();
      actor.telemetry();
      actor.physics_body.UpdateStateVector(0.0, phsCycle, Integrator.rk4);
      actor.mesh.position.set(
        actor.physics_body.position.x,
        actor.physics_body.position.y,
        actor.physics_body.position.z);
      actor.mesh.setRotationFromQuaternion(
          actor.physics_body.quaternion);
    }
////////////////////////////////////////////////////////////////////////////////
    function update(){
      if(!main.pauseSim){
        physTick();
        physTick();
      }
      updateThrustVectors(actor);  
      controls.update();
    }

////////////////////////////////////////////////////////////////////////////////
    function updateThrustVectors(actor){
      let dirUp = new THREE.Vector3(0, 1, 0);
      let dirDown = new THREE.Vector3(0, -1, 0);
      let indicators = [actor.mesh.getObjectByName("throttleIndicatorLF"),
                        actor.mesh.getObjectByName("throttleIndicatorRF"),
                        actor.mesh.getObjectByName("throttleIndicatorLB"),
                        actor.mesh.getObjectByName("throttleIndicatorRB")];
      let throttles = [actor.thrust.LF, 
                       actor.thrust.RF, 
                       actor.thrust.LB, 
                       actor.thrust.RB];
      let vel = new THREE.Vector3(actor.velocity.x, 
                                  actor.velocity.y,
                                  actor.velocity.z);
      let desVel = new THREE.Vector3(-actor.pidMatrix.roll.master.c,
                                     actor.pidMatrix.gas.master.c,
                                     actor.pidMatrix.pitch.master.c);
      let velIndicator = actor.mesh.getObjectByName("velIndicator");
      let setPointIndicator = actor.mesh.getObjectByName("setPointIndicator");
      if(vel.length() > 1.0E-5){
        velIndicator.visible = true;
        velIndicator.setDirection(vel);
        velIndicator.setLength(10*vel.length());
        velIndicator.applyQuaternion(actor.mesh.quaternion.clone().conjugate());
      }else{
        velIndicator.visible = false;
      }

      if(desVel.length() > 1.0E-5){
        setPointIndicator.visible = true;
        setPointIndicator.setDirection(desVel);
        setPointIndicator.setLength(10*desVel.length());
        setPointIndicator.applyQuaternion(
          actor.mesh.quaternion.clone().conjugate());
      }else{
        setPointIndicator.visible = false;
      }

      for (var i=0; i< throttles.length; i++){
        if (throttles[i] > 1.0E-5){
          indicators[i].visible = true;
          indicators[i].setDirection(dirUp);
          indicators[i].setLength(10*Math.abs(throttles[i]));
        }else if( Math.abs(throttles[i]) < 1.0E-5){
          indicators[i].visible = false;
        }else {
          indicators[i].visible = true;
          indicators[i].setDirection(dirDown);
          indicators[i].setLength(10*Math.abs(throttles[i]));
        }
      }
    }
////////////////////////////////////////////////////////////////////////////////
    function render(){
      //three js render events


      controls.target = new THREE.Vector3(actor.mesh.position.x,
                                          actor.mesh.position.y,
                                          actor.mesh.position.z);

      renderer.render(scene, camera);
    }
///////////////////////////////////////////////////////////////////////////////
    function animate(apply){
      requestAnimationFrame(animate);
      frmTime = new Date().getTime();
      if (frmTime - startCounter >= 33){
        startCounter = new Date().getTime();
        update();
        render();
        if(apply){
          $scope.$apply();
        }
      }
    }
////////////////////////////////////////////////////////////////////////////////
    function onWindowResize() {
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
    }
////////////////////////////////////////////////////////////////////////////////
    function onDocumentKeyDown(event){
      let keyCode = event.which;
      switch (keyCode){
        case 17://Ctrl
          controls.enabled = false;
          break;
        case 80://P
          main.pauseSim = !main.pauseSim;
          break;
        case 72://H
          main.help = !main.help;
          main.toggleHelp();
        case 81://Q
          setYaw +=0.2;
          actor.setPoint[3] = setYaw;
          break;
        case 69://E
          setYaw -=0.2;
          actor.setPoint[3] = setYaw;
          break;
        case 187://+
          setAlt += 0.25;
          actor.setPoint[0] = setAlt;
          break;
        case 187://-
          setAlt -= 0.25;
          actor.setPoint[0] = setAlt;
          break;
      }
    }

    function onDocumentKeyUp(event){
      let keyCode = event.which;
      switch (keyCode){
        case 17://Ctrl
          controls.enabled = true;
          break;
      }
    }
////////////////////////////////////////////////////////////////////////////////
   function buildPIDMatrix(){
     return {gas:   {master: {c: 0, er: 0, integ: 0}, 
                     second: {c: 0, er: 0, integ: 0},
                      third: {c: 0, er: 0, integ: 0}},
             pitch: {master: {c: 0, er: 0, integ: 0}, 
                     second: {c: 0, er: 0, integ: 0}, 
                      third: {c: 0, er: 0, integ: 0}},
             roll:  {master: {c: 0, er: 0, integ: 0}, 
                     second: {c: 0, er: 0, integ: 0}, 
                      third: {c: 0, er: 0, integ: 0}},
             yaw:   {master: {c: 0, er: 0, integ: 0}, 
                     second: {c: 0, er: 0, integ: 0},
                      third: {c: 0, er: 0, integ: 0}}};
    }
});
////////////////////////////////////////////////////////////////////////////////
  function buildActorMesh(scale = {x: 1, y: 1, z: 1}){
    var actorAxesHelper = new THREE.AxesHelper( 3 );
    let actorModel = new THREE.Group();

    //engine und engine throttle indicators          
    let engineLF = new THREE.Mesh(
      new THREE.SphereGeometry(1, 10, 5),
      new THREE.MeshLambertMaterial({color: 0x504948, reflectivity: 0.3}));
    let engineRF = new THREE.Mesh(
      new THREE.SphereGeometry(1, 10, 5),
      new THREE.MeshLambertMaterial({color: 0x504948, reflectivity: 0.3}));
    let engineLB = new THREE.Mesh(
      new THREE.SphereGeometry(1, 10, 5),
      new THREE.MeshLambertMaterial({color: 0x504948, reflectivity: 0.3}));
    let engineRB = new THREE.Mesh(
      new THREE.SphereGeometry(1, 10, 5),
      new THREE.MeshLambertMaterial({color: 0x504948, reflectivity: 0.3}));
////////////////////////////////////////////////////////////////////////////////
    engineLF.position.x = 1, engineLF.position.y = 0, engineLF.position.z = 1;
    engineRF.position.x = -1, engineRF.position.y = 0, engineRF.position.z = 1;
    engineLB.position.x = 1, engineLB.position.y = 0, engineLB.position.z = -1;
    engineRB.position.x = -1, engineRB.position.y = 0, engineRB.position.z = -1;
    //THREE.ArrowHelper(direction, origin, length, color)
    let throttleIndicatorLF = new THREE.ArrowHelper( 
      new THREE.Vector3( 0, 1, 0 ),
      new THREE.Vector3( 0, 0, 0 ), 3, 0x00ff00 );
    let throttleIndicatorRF = new THREE.ArrowHelper( 
      new THREE.Vector3( 0, 1, 0 ),
      new THREE.Vector3( 0, 0, 0 ), 3, 0x0000ff );
    let throttleIndicatorLB = new THREE.ArrowHelper( 
      new THREE.Vector3( 0, 1, 0 ),
      new THREE.Vector3( 0, 0, 0 ), 3, 0xffa500 );
    let throttleIndicatorRB = new THREE.ArrowHelper( 
      new THREE.Vector3( 0, 1, 0 ),
      new THREE.Vector3( 0, 0, 0 ), 3, 0x551a8b );
    //
    throttleIndicatorLF.position.x = engineLF.position.x; 
    throttleIndicatorLF.position.y = engineLF.position.y;
    throttleIndicatorLF.position.z = engineLF.position.z;
    throttleIndicatorLF.name = "throttleIndicatorLF";
    //
    throttleIndicatorRF.position.x = engineRF.position.x; 
    throttleIndicatorRF.position.y = engineRF.position.y;
    throttleIndicatorRF.position.z = engineRF.position.z;
    throttleIndicatorRF.name = "throttleIndicatorRF";
    //
    throttleIndicatorLB.position.x = engineLB.position.x; 
    throttleIndicatorLB.position.y = engineLB.position.y;
    throttleIndicatorLB.position.z = engineLB.position.z;
    throttleIndicatorLB.name = "throttleIndicatorLB";
    //
    throttleIndicatorRB.position.x = engineRB.position.x; 
    throttleIndicatorRB.position.y = engineRB.position.y;
    throttleIndicatorRB.position.z = engineRB.position.z;
    throttleIndicatorRB.name = "throttleIndicatorRB";
    engineLF.scale.y = 0.35;
    engineRF.scale.y = 0.35;
    engineLB.scale.y = 0.35;
    engineRB.scale.y = 0.35;
    actorModel.add(engineLF);
    actorModel.add(throttleIndicatorLF);
    actorModel.add(engineRF);
    actorModel.add(throttleIndicatorRF);
    actorModel.add(engineLB);
    actorModel.add(throttleIndicatorLB);
    actorModel.add(engineRB);
    actorModel.add(throttleIndicatorRB);
//    actorModel.add(actorAxesHelper);

    let velIndicator = new THREE.ArrowHelper(new THREE.Vector3(0,1,0),
                                             new THREE.Vector3(),
                                             1, 0x00ff00);
    let setPointIndicator = new THREE.ArrowHelper(new THREE.Vector3(0,-1,0),
                                             new THREE.Vector3(),
                                             1, 0xff0000);
    velIndicator.name = "velIndicator";
    setPointIndicator.name = "setPointIndicator"

    actorModel.add(velIndicator);
    actorModel.add(setPointIndicator);
    //
    actorModel.scale.x=scale.x;
    actorModel.scale.y=scale.y;
    actorModel.scale.z=scale.z;
    return actorModel;
  }

     
      
      
      
   
