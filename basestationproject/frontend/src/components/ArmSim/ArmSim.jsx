import React, { useRef, useEffect } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import URDFLoader from "urdf-loader";

//
// ─── ArmScene ───────────────────────────────────────────────────────────────────
//
function ArmScene({ jointAngles }) {
  const robotRef = useRef(null);
  const { scene } = useThree();

  // List your exact URDF joint names in the same order as jointAngles[0..5]:
  const jointNames = [
    "joint1",       // index 0
    "joint2",       // index 1
    "joint3",       // index 2
    "joint4",       // index 3
    "joint5",       // index 4
  ];

  // Load the URDF once
  useEffect(() => {
    const loader = new URDFLoader();
    loader.packages = {
      // Adjust “arm_description” if your URDF uses a different package name
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

    // Clean up on unmount
    return () => {
      if (robotRef.current) {
        scene.remove(robotRef.current);
      }
    };
  }, [scene]);

  // Every frame, set joint values based on jointAngles prop
  useFrame(() => {
    const robot = robotRef.current;
    if (!robot) return;

    jointNames.forEach((name, idx) => {
      const angleDeg = jointAngles[idx];
      if (angleDeg == null) return;

      const joint = robot.joints[name];
      if (!joint) {
        console.warn(`ArmSim: cannot find URDF joint '${name}'`);
        return;
      }

      const angleRad = (angleDeg * Math.PI) / 180;
      joint.setJointValue(angleRad);
    });
  });

  return null;
}

//
// ─── ArmSim Canvas Wrapper ──────────────────────────────────────────────────────
//
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
