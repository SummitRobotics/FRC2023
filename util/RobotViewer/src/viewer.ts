import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { GUI } from 'dat.gui';

declare const window: any;

/* TODO
* [ ] move definitions of primitives to a table/object
* [ ] generic binding to ncclient keys
* [ ] get build flow improved
* [ ] update readme
*/

const inchesToMeters = (inches: number) => {
    return inches * 0.0254;
};

const feetToMeters = (feet: number) => {
    return feet * 0.3048;
};

const degToRad = (deg: number) => {
    return deg * Math.PI / 180.0;
};

// Create Scene and Renderer
const scene = new THREE.Scene();
const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Create Camera / Controls
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.z = 2;
const controls = new OrbitControls(camera, renderer.domElement);

// Create helpers
const loader = new THREE.TextureLoader();

// Add a grid to the scene
const size = 20;
const divisions = 40;
const gridHelper = new THREE.GridHelper(size, divisions);
scene.add(gridHelper);

const finalizeObject = (geometry: THREE.BufferGeometry, options: any = {}) => {
    const materialOptions = {
        color: options.color ?? 0x00ff00,
        wireframe: options.wireframe ?? true,
    } as any;
    if (options.texture) {
        materialOptions.map = loader.load(options.texture);
    }
    const material = new THREE.MeshBasicMaterial(materialOptions);
    const obj = new THREE.Mesh(geometry, material);
    obj.add(new THREE.AxesHelper(0.1));
    return obj;
}

const createBox = (width: number, height: number, depth: number, options: any = {}) => {
    const box = new THREE.BoxGeometry(width, height, depth);
    return finalizeObject(box, options);
}

const createCylinder = (radius: number, height: number, options: any = {}) => {
    const cylinder = new THREE.CylinderGeometry(radius, radius, height, options.segments ?? 20);
    return finalizeObject(cylinder, options);
}

/**
 * Hierarchy of joints. +X forward, +Y right, +Z up. Default pose has arm
 * segments vertical. Turret is back from robot base to accommodate the forward
 * intake.
 *  base \
 *      turret \
 *          arm1 \
 *              arm2 \
 *                  arm3: \
 *                      grabber
 */

// Field: 54.27083 x 26.2916 ft ((TODO: offset correctly per .json config)
const field = createBox(feetToMeters(54.27083), 0, feetToMeters(26.2916), {
    texture: '../assets/2023-field.png',
    wireframe: false,
    color: 0xffffff,
});
scene.add(field);

// Base: 32x32x9" origin at center bottom
const base = createBox(inchesToMeters(32), inchesToMeters(6), inchesToMeters(32));
base.geometry.translate(0, inchesToMeters(3), 0);
scene.add(base);

// Turret: 8x2" origin at center bottom, offset forward
const turret = createCylinder(inchesToMeters(8), inchesToMeters(2));
turret.geometry.translate(0, inchesToMeters(-1), 0);
turret.position.set(inchesToMeters(-8), inchesToMeters(8), 0);
base.add(turret);

// Arm1: 31" long, origin at pivot with Turret, rotated vertically
const arm1 = createBox(inchesToMeters(31), inchesToMeters(4), inchesToMeters(4), { color: 0xff0000 });
arm1.geometry.rotateZ(Math.PI / 2);
arm1.geometry.translate(0, inchesToMeters(15.5), inchesToMeters(0));
turret.add(arm1);

// Arm2: 29" long, origin at pivot with Arm1, rotated vertically
const arm2 = createBox(inchesToMeters(29), inchesToMeters(4), inchesToMeters(4), { color: 0x0000ff });
arm2.geometry.rotateZ(Math.PI / 2);
arm2.geometry.translate(0, inchesToMeters(14.5), 0);
arm2.position.set(0, inchesToMeters(31), 0);
arm1.add(arm2);

// Arm3: 18" long, origin at pivot with Arm2 (63deg up, ~90deg down)
const arm3 = createBox(inchesToMeters(18), inchesToMeters(4), inchesToMeters(4))
arm3.geometry.rotateZ(Math.PI / 2);
arm3.geometry.translate(0, inchesToMeters(9), 0);
arm3.position.set(0, inchesToMeters(29), 0);
arm2.add(arm3);


// Create GUI for viewing/tweaking values
const gui = new GUI();
gui.close();

const baseFolder = gui.addFolder('Base');
baseFolder.add(base.rotation, 'y', -Math.PI, Math.PI, 0.01).name('Rotation');
baseFolder.open();

const turretFolder = gui.addFolder('Turret');
turretFolder.add(turret.rotation, 'y', -Math.PI, Math.PI, 0.01).name('Rotation');
turretFolder.open();

const arm1Folder = gui.addFolder('Arm1');
arm1Folder.add(arm1.rotation, 'z', -Math.PI, Math.PI, 0.01).name('Angle');
arm1Folder.open();

const arm2Folder = gui.addFolder('Arm2');
arm2Folder.add(arm2.rotation, 'z', -Math.PI, Math.PI, 0.01).name('Angle');
arm2Folder.open();

const arm3Folder = gui.addFolder('Arm3');
arm3Folder.add(arm3.rotation, 'z', -Math.PI, Math.PI, 0.01).name('Angle');
arm3Folder.open();

const updateGuiControllers = () => {
    // Force update of all GUI controllers
    for (let i = 0; i < Object.keys(gui.__folders).length; i++) {
        const key = Object.keys(gui.__folders)[i];
        for (let j = 0; j < gui.__folders[key].__controllers.length; j++) {
            gui.__folders[key].__controllers[j].updateDisplay();
        }
    }
};

let resetCount = 0;
const controller = {
    reset: () => {
        base.rotation.y = 0;
        turret.rotation.y = 0;
        arm1.rotation.z = 0;
        arm2.rotation.z = 0;
        arm3.rotation.z = 0;
        updateGuiControllers();

        // Send to main/ntClient (example)
        window.electronAPI.set('resetCount', resetCount++);
    },
};

const actionsFolder = gui.addFolder('Actions');
actionsFolder.add(gridHelper, 'visible').name('Toggle Grid');
actionsFolder.add(controller, 'reset').name('Reset');
actionsFolder.open();

// Configure interprocess hooks
window.electronAPI.onUpdate((_: any, key: string, value: any, valueType: string, type: string, id: number, flags: number) => {
    // TODO: Get proper keys and bind to appropriate controls
    let handled = true;
    if (key === '/SmartDashboard/Arm/turretAngle') {
        turret.rotation.y = -value;
    } else if (key === '/SmartDashboard/Arm/firstJointAngle') {
        arm1.rotation.z = -value;
    } else if (key === '/SmartDashboard/Arm/secondJointAngle') {
        arm2.rotation.z = -value;
    } else if (key === '/SmartDashboard/Arm/thirdJointAngle') {
        arm3.rotation.z = -value;
    } else if (key === '/SmartDashboard/Arm/wristAngle') {
        // TODO
    } else if (key === '/SmartDashboard/Arm/grabberClamp') {
        // TODO
    } else {
        handled = false;
    }
    if (handled) {
        updateGuiControllers();
    }
    console.log(`update: ${key} - ${value} - ${valueType} - ${type} - ${id} - ${flags}`);
});



// Handle window resize event
window.addEventListener('resize', onWindowResize, false);

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
    render();
}

// Setup animation loop
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    render();
}

function render() {
    renderer.render(scene, camera);
}

animate();
