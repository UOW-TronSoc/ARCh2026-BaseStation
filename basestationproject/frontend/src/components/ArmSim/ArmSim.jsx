// src/components/ArmSim/ArmSim.jsx
import React, { useRef, useEffect } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import URDFLoader from "urdf-loader";

function ArmScene({ jointAngles }) {
  const robotRef = useRef();
  const { scene } = useThree();

  useEffect(() => {
    const loader = new URDFLoader();
    loader.packages = {
      arm_description: "/urdf"
    };

    loader.load("/urdf/robot.urdf", (robot) => {
      robotRef.current = robot;
      robot.traverse((obj) => {
        obj.castShadow = true;
        obj.receiveShadow = true;
      });
      scene.add(robot);
    });
  }, [scene]);

  useFrame(() => {
    if (!robotRef.current) return;
    const jointNames = Object.keys(robotRef.current.joints).reverse();
    // reversing the joints control //gripper doesn't work

    jointNames.forEach((name, i) => {
      const joint = robotRef.current.joints[name];
      if (joint && jointAngles[i] !== undefined) {
        joint.setJointValue((jointAngles[i] * Math.PI) / 180);
      }
    });
  });

  return null;
}

export default function ArmSim({ jointAngles }) {
  return (
    <Canvas shadows camera={{ position: [2, 2, 4], fov: 50 }}>
      <ambientLight intensity={0.6} />
      <directionalLight position={[5, 5, 5]} castShadow />
      <OrbitControls />
      <ArmScene jointAngles={jointAngles} />
    </Canvas>
  );
}
