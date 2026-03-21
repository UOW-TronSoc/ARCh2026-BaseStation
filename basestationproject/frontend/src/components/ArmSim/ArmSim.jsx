import React, { useRef, useEffect } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import URDFLoader from "urdf-loader";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import * as THREE from "three";

//
// ─── ArmScene ───────────────────────────────────────────────────────────────────
//
function ArmScene({ jointAngles }) {
  const robotRef = useRef(null);
  const { scene } = useThree();

  // URDF joint names (J1-J5)
  const jointNames = ["J1", "J2", "J3", "J4", "J5"];

  // Load the URDF once
  useEffect(() => {
    const loader = new URDFLoader();
    const manager = new THREE.LoadingManager();
    const objLoader = new OBJLoader(manager);
    
    // Set the working path for resolving relative mesh paths
    loader.workingPath = "/urdf/";

    // Custom mesh loading callback for OBJ files
    loader.loadMeshCb = (path, manager, onComplete) => {
      console.log("Loading mesh:", path);
      objLoader.load(
        path,
        (obj) => {
          console.log("✅ Mesh loaded:", path);
          onComplete(obj);
        },
        undefined,
        (error) => {
          console.error("❌ Error loading mesh:", path, error);
          onComplete(null);
        }
      );
    };

    console.log("Loading URDF from /urdf/robot.urdf");
    
    loader.load(
      "/urdf/robot.urdf",
      (robot) => {
        console.log("✅ URDF loaded successfully!", robot);
        console.log("Available joints:", Object.keys(robot.joints));
        robotRef.current = robot;

        const linkColors = {
          link_0: new THREE.Color(0.866667, 0.321569, 0.156863),
          link_1: new THREE.Color(0.909804, 0.572549, 0.164706),
          link_2: new THREE.Color(1.0, 0.756863, 0.054902),
          link_3: new THREE.Color(0.372549, 0.654902, 0.239216),
          link_4: new THREE.Color(0.0862745, 0.317647, 0.690196),
          link_5: new THREE.Color(0.368627, 0.25098, 0.643137),
          fre_gripper__fixed: new THREE.Color(0.596078, 0.121569, 0.67451),
        };

        // Rotate robot so base is horizontal (rotate -90° around X-axis)
        robot.rotation.x = -Math.PI / 2;

        robot.traverse((obj) => {
          obj.castShadow = true;
          obj.receiveShadow = true;

          if (obj.isMesh) {
            const parentName = obj.parent?.name;
            const color =
              linkColors[parentName] || linkColors[obj.name] || null;

            if (color) {
              // Clone material so we don't mutate shared materials
              obj.material = obj.material.clone();
              obj.material.color.copy(color);
            }
          }
        });
        scene.add(robot);
      },
      (progress) => {
        console.log("URDF loading progress:", progress);
      },
      (error) => {
        console.error("❌ Error loading URDF:", error);
      }
    );

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

    // Update joints - expect 5 joints (J1-J5)
    jointNames.forEach((name, idx) => {
      const angleDeg = jointAngles[idx];
      if (angleDeg == null || idx >= 5) return; // Only handle first 5 joints

      const joint = robot.joints[name];
      if (!joint) {
        console.warn(`ArmSim: cannot find URDF joint '${name}'`);
        return;
      }

      // Convert degrees to radians for URDF joint values
      const angleRad = (angleDeg * Math.PI) / 180;
      joint.setJointValue(angleRad);
    });
  });

  return null;
}

//
// ─── Ground Plane ───────────────────────────────────────────────────────────────
//
function GroundPlane() {
  return (
    <group>
      {/* Grid helper for spatial reference */}
      <gridHelper args={[4, 20, 0x444444, 0x222222]} rotation={[0, 0, 0]} />
      {/* Solid ground plane */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.01, 0]} receiveShadow>
        <planeGeometry args={[4, 4]} />
        <meshStandardMaterial color={0x1a1a1a} />
      </mesh>
    </group>
  );
}

//
// ─── ArmSim Canvas Wrapper ──────────────────────────────────────────────────────
//
export default function ArmSim({ jointAngles }) {
  return (
    <Canvas shadows camera={{ position: [1.5, 1.5, 1.5], fov: 50 }}>
      {/* Lighting */}
      <ambientLight intensity={0.5} />
      <directionalLight position={[5, 10, 5]} intensity={1} castShadow />
      <directionalLight position={[-5, 5, -5]} intensity={0.3} />

      {/* Full 3D orbit controls like RaiSim */}
      <OrbitControls
        enableDamping
        dampingFactor={0.1}
        target={[0, 0.3, 0]}
        minDistance={0.5}
        maxDistance={5}
      />

      {/* Ground plane and grid */}
      <GroundPlane />

      {/* Robot arm */}
      <ArmScene jointAngles={jointAngles} />
    </Canvas>
  );
}
