import React, { useRef, useEffect } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import URDFLoader from "urdf-loader";
import { Box3, Vector3 } from "three";

function ArmScene({ jointAngles, controlsRef }) {
  const robotRef = useRef();
  const { scene } = useThree();

  useEffect(() => {
    const loader = new URDFLoader();
    loader.packages = { arm_description: "/urdf" };
    loader.load("/urdf/robot.urdf", (robot) => {
      robotRef.current = robot;
      robot.traverse(obj => {
        obj.castShadow = true;
        obj.receiveShadow = true;
      });
      scene.add(robot);

      // center the OrbitControls on the robot
      const box = new Box3().setFromObject(robot);
      const c   = new Vector3();
      box.getCenter(c);
      if (controlsRef.current) {
        controlsRef.current.target.copy(c);
        controlsRef.current.update();
      }
    });
  }, [scene, controlsRef]);

  useFrame(() => {
    if (!robotRef.current) return;
    const names = Object.keys(robotRef.current.joints).reverse();
    names.forEach((name, i) => {
      const rad = (jointAngles[i] * Math.PI) / 180;
      robotRef.current.joints[name].setJointValue(rad);
    });
  });

  return null;
}

export default function ArmSim({ jointAngles }) {
  const controlsRef = useRef();
  return (
    <Canvas
      shadows
      camera={{ position:[2,2,4], fov:50 }}
      onPointerDown={e => {
        if (controlsRef.current) {
          controlsRef.current.target.copy(e.point);
          controlsRef.current.update();
        }
      }}
    >
      <ambientLight intensity={0.6}/>
      <directionalLight position={[5,5,5]} castShadow/>
      <OrbitControls ref={controlsRef}/>
      <ArmScene jointAngles={jointAngles} controlsRef={controlsRef}/>
    </Canvas>
  );
}
