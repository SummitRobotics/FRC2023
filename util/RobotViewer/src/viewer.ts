import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { GUI } from 'dat.gui';

const DEBUG = false;

declare const window: any;

/* TODO
* [ ] move definitions of primitives to a table/object
* [ ] generic binding to ncclient keys
* [ ] get build flow improved
* [ ] update readme
* [ ] get proper bindings to networktable controls

- https://threejs.org/docs/index.html#api/en/geometries/CircleGeometry
- https://threejs.org/docs/index.html#api/en/geometries/RingGeometry
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
renderer.shadowMap.enabled = true;
document.body.appendChild(renderer.domElement);

// Create Camera / Controls
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.z = 2;
const controls = new OrbitControls(camera, renderer.domElement);
controls.maxPolarAngle = Math.PI / 2;

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
        wireframe: DEBUG ? true : options.wireframe ?? false,
        side: options.side ?? THREE.FrontSide,
    } as any;
    if (options.texture) {
        materialOptions.map = options.texture;
    }
    const material = new THREE.MeshPhongMaterial(materialOptions);
    const obj = new THREE.Mesh(geometry, material);
    obj.receiveShadow = options.receiveShadow ?? true;
    obj.castShadow = options.castShadow ?? true;
    if (DEBUG) {
        obj.add(new THREE.AxesHelper(0.1));
    }
    return obj;
}

const createPlane = (width: number, height: number, options: any = {}) => {
    const plane = new THREE.PlaneGeometry(width, height, options.widthSegments ?? 32, options.heightSegments ?? 32);
    return finalizeObject(plane, options);
}

const createBox = (width: number, height: number, depth: number, options: any = {}) => {
    const box = new THREE.BoxGeometry(width, height, depth);
    return finalizeObject(box, options);
}

const createCylinder = (radius: number, height: number, options: any = {}) => {
    const cylinder = new THREE.CylinderGeometry(radius, radius, height, options.segments ?? 20);
    return finalizeObject(cylinder, options);
}

// Add some lighting to the scene
const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
scene.add(ambientLight);

const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
dirLight.position.set(10, 20, 0);
dirLight.castShadow = true;

// Set up shadow properties for the light (all defaults)
dirLight.shadow.mapSize.width = 512;
dirLight.shadow.mapSize.height = 512;
dirLight.shadow.camera.near = 0.5;
dirLight.shadow.camera.far = 500;
scene.add(dirLight);

/**
 * Field.
 */

/** 
 * TODO: figure out why this imoprt fails for others. for now, just inline the config.
 */
// import fieldConfig from '../assets/2023-chargedup.json' assert {type: 'json'};
const fieldConfig = {
    "game": "Charged Up",
    "field-image": "2023-field.png",
    "field-corners": {
        "top-left": [
            46,
            36
        ],
        "bottom-right": [
            1088,
            544
        ]
    },
    "field-size": [
        54.27083,
        26.2916
    ],
    "field-unit": "foot"
};

loader.load(`../assets/${fieldConfig['field-image']}`, (texture) => {
    const [left, top] = fieldConfig['field-corners']['top-left'];
    const [right, bottom] = fieldConfig['field-corners']['bottom-right'];

    // check assumption that corners are centered in image
    if ((top !== (texture.image.naturalHeight - bottom)) ||
        (left !== (texture.image.naturalWidth - right))) {
        throw new Error('Field not centered in image')
    }

    let [fieldWidth, fieldHeight] = fieldConfig['field-size'];
    fieldWidth = feetToMeters(fieldWidth);
    fieldHeight = feetToMeters(fieldHeight);

    const field = createPlane(fieldWidth, fieldHeight, {
        texture: texture,
        wireframe: false,
        color: 0xffffff,
        castShadows: false,
        receiveShadows: true,
    });

    // Scale field (dimensions are play area only)
    field.geometry.scale(texture.image.naturalWidth / (texture.image.naturalWidth - 2 * left),
        texture.image.naturalHeight / (texture.image.naturalHeight - 2 * top), 1);
    field.geometry.rotateX(-Math.PI / 2);
    scene.add(field);

    // Create edges of play area
    const edgeHeight = feetToMeters(0.5);
    const edgeColor = 0xff00ff;

    let edge = createPlane(fieldWidth, edgeHeight, { color: edgeColor, side: THREE.DoubleSide, });
    edge.geometry.translate(0, edgeHeight / 2, fieldHeight * 0.5);
    scene.add(edge);

    edge = createPlane(fieldWidth, edgeHeight, { color: edgeColor, side: THREE.DoubleSide, });
    edge.geometry.translate(0, edgeHeight / 2, -fieldHeight * 0.5);
    scene.add(edge);

    edge = createPlane(fieldHeight, edgeHeight, { color: edgeColor, side: THREE.DoubleSide, });
    edge.geometry.translate(0, edgeHeight / 2, fieldWidth * 0.5);
    edge.geometry.rotateY(Math.PI / 2);
    scene.add(edge);

    edge = createPlane(fieldHeight, edgeHeight, { color: edgeColor, side: THREE.DoubleSide, });
    edge.geometry.translate(0, edgeHeight / 2, fieldWidth * 0.5);
    edge.geometry.rotateY(-Math.PI / 2);
    scene.add(edge);

    // const points = [];
    // points.push(new THREE.Vector3(-fieldWidth * 0.5, -fieldHeight * 0.5, 0));
    // points.push(new THREE.Vector3(fieldWidth * 0.5, -fieldHeight * 0.5, 0));
    // points.push(new THREE.Vector3(fieldWidth * 0.5, fieldHeight * 0.5, 0));
    // points.push(new THREE.Vector3(-fieldWidth * 0.5, fieldHeight * 0.5, 0));
    // points.push(new THREE.Vector3(-fieldWidth * 0.5, -fieldHeight * 0.5, 0));
    // const geometry = new THREE.BufferGeometry().setFromPoints(points);
    // const material = new THREE.LineBasicMaterial({ color: 0x0000ff });
    // const playEdge = new THREE.Line(geometry, material);
    // playEdge.geometry.rotateX(-Math.PI / 2);
    // scene.add(playEdge);
});

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

// Base: 32x32x9" origin at center bottom
const base = createBox(inchesToMeters(32), inchesToMeters(6), inchesToMeters(32));
base.geometry.translate(0, inchesToMeters(3), 0);
scene.add(base);

// Turret: 8x2" origin at center bottom, offset forward
const turret = createCylinder(inchesToMeters(8), inchesToMeters(2), { color: 0xffff00 });
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

// Arm3: 18" long, origin at pivot with Arm2 (range of motion: 63deg up, ~90deg down)
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
turretFolder.add(turret.rotation, 'y', degToRad(-135), degToRad(135), 0.01).name('Rotation');
turretFolder.open();

const arm1Folder = gui.addFolder('Arm1');
arm1Folder.add(arm1.rotation, 'z', degToRad(-60), degToRad(10), 0.01).name('Angle');
arm1Folder.open();

const arm2Folder = gui.addFolder('Arm2');
arm2Folder.add(arm2.rotation, 'z', degToRad(-60), degToRad(10), 0.01).name('Angle');
arm2Folder.open();

const arm3Folder = gui.addFolder('Arm3');
arm3Folder.add(arm3.rotation, 'z', degToRad(-60), degToRad(10), 0.01).name('Angle');
arm3Folder.open();

// Default camera to look at our bot
camera.position.set(3, 3, 3);
camera.lookAt(base.position);

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
    // Coerce NaN value to zero
    if (value !== value) {
        value = 0;
    }
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
    if (DEBUG) {
        console.log(`update: ${key} - ${value} - ${valueType} - ${type} - ${id} - ${flags}`);
    }
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
