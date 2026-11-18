import * as THREE from 'https://cdn.skypack.dev/three@0.157.0';
import { GLTFLoader } from 'https://cdn.skypack.dev/three@0.157.0/examples/jsm/loaders/GLTFLoader.js';

let rocket = null;
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth/window.innerHeight, 0.1, 1000);
camera.position.z = 5;

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Lighting
scene.add(new THREE.AmbientLight(0xffffff, 0.4));
const light = new THREE.DirectionalLight(0xffffff, 1);
light.position.set(5, 5, 5);
scene.add(light);

// Load rocket model
const targetQuat = new THREE.Quaternion();
const loader = new GLTFLoader();

loader.load('submarine_torpedo_weapon.glb', (gltf) => {
  rocket = gltf.scene;
  rocket.scale.set(1.5, 1.5, 1.5);
  rocket.rotation.x = Math.PI / 2;
  scene.add(rocket);
}, undefined, (err) => {
  console.error('Failed to load model:', err);
});

// Simulate IMU data
function updateTargetQuaternion(roll, pitch, yaw) {
  const euler = new THREE.Euler(
    THREE.MathUtils.degToRad(pitch),
    THREE.MathUtils.degToRad(yaw),
    THREE.MathUtils.degToRad(roll),
    'ZYX'
  );
  targetQuat.setFromEuler(euler);
}

setInterval(() => {
  const roll = (Math.random() * 60 - 30).toFixed(1);
  const pitch = (Math.random() * 60 - 30).toFixed(1);
  const yaw = (Math.random() * 360 - 180).toFixed(1);

  document.getElementById("roll").textContent = roll;
  document.getElementById("pitch").textContent = pitch;
  document.getElementById("yaw").textContent = yaw;

  updateTargetQuaternion(roll, pitch, yaw);
}, 500);

function animate() {
  requestAnimationFrame(animate);
  if (rocket) rocket.quaternion.slerp(targetQuat, 0.02);
  renderer.render(scene, camera);
}
animate();

window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
