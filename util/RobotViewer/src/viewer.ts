import * as THREE from 'three';
import { GUI } from 'dat.gui';
// import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
// @ts-ignore
import { OrbitControls } from "./3rdparty/ThreeOrbitControlsGizmo/OrbitControls.js";
// @ts-ignore
import { OrbitControlsGizmo } from "./3rdparty/ThreeOrbitControlsGizmo/OrbitControlsGizmo.js";

const DEBUG = false;

declare const window: any;

/* TODO
* [ ] move definitions of primitives to a table/object
* [ ] generic binding to ncclient keys
* [ ] get build flow improved
* [ ] update readme
* [ ] get proper bindings to networktable controls
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
const controlsGizmo = new OrbitControlsGizmo(controls, { size: 100, padding: 8 });
document.body.appendChild(controlsGizmo.domElement);
controlsGizmo.domElement.hidden = true;

// Add a grid to the scene
const size = 20;
const divisions = 40;
const gridHelper = new THREE.GridHelper(size, divisions);
gridHelper.visible = false;
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

const createObj = () => {
    const obj = new THREE.Object3D();
    if (DEBUG) {
        obj.add(new THREE.AxesHelper(0.1));
    }
    return obj;
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
 * TODO: figure out why this import fails for others. for now, just inline the config.
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

    // Create a little edge of the play area (visible only from inside)
    const edgeHeight = inchesToMeters(3);
    const edgeColor = 0x60a9b2;

    let edge = createPlane(fieldWidth, edgeHeight, { color: edgeColor });
    edge.geometry.rotateY(Math.PI);
    edge.geometry.translate(0, edgeHeight / 2, fieldHeight * 0.5);
    scene.add(edge);

    edge = createPlane(fieldWidth, edgeHeight, { color: edgeColor });
    edge.geometry.translate(0, edgeHeight / 2, -fieldHeight * 0.5);
    scene.add(edge);

    edge = createPlane(fieldHeight, edgeHeight, { color: edgeColor });
    edge.geometry.rotateY(Math.PI);
    edge.geometry.translate(0, edgeHeight / 2, fieldWidth * 0.5);
    edge.geometry.rotateY(Math.PI / 2);
    scene.add(edge);

    edge = createPlane(fieldHeight, edgeHeight, { color: edgeColor });
    edge.geometry.rotateY(Math.PI);
    edge.geometry.translate(0, edgeHeight / 2, fieldWidth * 0.5);
    edge.geometry.rotateY(-Math.PI / 2);
    scene.add(edge);
});

/**
 * Hierarchy of joints (mixed with geometry). +X forward, +Y right, +Z up. Default
 * pose has arm segments vertical. Turret is back from robot base to accommodate the
 * forward intake.
 *  base \
 *      turret \
 *          joint1 \
 *              joint2 \
 *                  joint3 \
 *                      wrist \
 *                          grabber
 */

// Base: 31.5x27x6" origin at center bottom
const base = createBox(inchesToMeters(31.5), inchesToMeters(6), inchesToMeters(27));
base.geometry.translate(0, inchesToMeters(3), 0);
scene.add(base);

// Turret: 8x1.2" origin at center top, offset forward
const turret = createCylinder(inchesToMeters(8), inchesToMeters(1.2), { color: 0xffff00 });
turret.geometry.translate(0, inchesToMeters(-0.6), 0);
turret.position.set(-0.2413, inchesToMeters(7.2), 0);
base.add(turret);

// Turret Stand: 8"
const stand = createBox(inchesToMeters(5), inchesToMeters(8), inchesToMeters(5), { color: 0xcc33cc });
stand.position.y = inchesToMeters(4);
turret.add(stand);

// Joint1: 31" long arm, origin at pivot with Turret, rotated vertically
const joint1 = createBox(inchesToMeters(31), inchesToMeters(4), inchesToMeters(4), { color: 0xff0000 });
joint1.position.y = inchesToMeters(8);
joint1.geometry.rotateZ(Math.PI / 2);
joint1.geometry.translate(0, inchesToMeters(15.5), inchesToMeters(0));
turret.add(joint1);

// Joint2: 29" long arm, origin at pivot with Joint1, rotated vertically
const joint2 = createBox(inchesToMeters(29), inchesToMeters(4), inchesToMeters(4), { color: 0x0000ff });
joint2.geometry.rotateZ(Math.PI / 2);
joint2.geometry.translate(0, inchesToMeters(14.5), 0);
joint2.position.set(0, inchesToMeters(31), 0);
joint1.add(joint2);

// Joint3: start of the grabber mechanism
const joint3 = createObj();
joint3.position.set(0, inchesToMeters(29), 0);
joint2.add(joint3);

// Wrist: origin at joint3
const wrist = createCylinder(inchesToMeters(4), inchesToMeters(1), { color: 0x00ffff });
wrist.geometry.translate(0, inchesToMeters(0.5), 0);
joint3.add(wrist);

// Grabber: origin atop wrist, 17" in total length (4" base, 13" clamps)
const grabber = createObj();
grabber.position.set(0, inchesToMeters(1), 0);
wrist.add(grabber);

const grabberBase = createBox(inchesToMeters(4), inchesToMeters(3), inchesToMeters(3));
grabberBase.geometry.rotateZ(Math.PI / 2);
grabberBase.geometry.translate(0, inchesToMeters(2), 0);
grabber.add(grabberBase);

const grabberClamp1 = createBox(inchesToMeters(13), inchesToMeters(3), inchesToMeters(1), { color: 0xcc00cc });
grabberClamp1.geometry.rotateZ(Math.PI / 2);
grabberClamp1.geometry.translate(0, inchesToMeters(6.5), 0);
grabberClamp1.position.set(0, inchesToMeters(4), 0);
grabber.add(grabberClamp1);

const grabberClamp2 = createBox(inchesToMeters(13), inchesToMeters(3), inchesToMeters(1), { color: 0xcc00cc });
grabberClamp2.geometry.rotateZ(Math.PI / 2);
grabberClamp2.geometry.translate(0, inchesToMeters(6.5), 0);
grabberClamp2.position.set(0, inchesToMeters(4), 0);
grabber.add(grabberClamp2);

// Open/close the clamps. Just moves positions to 18" when open, 4" when closed
let clampOpen = false;
const setClampOpen = (open: boolean) => {
    if (open) {
        grabberClamp1.position.setZ(inchesToMeters(-9));
        grabberClamp2.position.setZ(inchesToMeters(9));
    } else {
        grabberClamp1.position.setZ(inchesToMeters(-2));
        grabberClamp2.position.setZ(inchesToMeters(2));
    }
    clampOpen = open;
}
setClampOpen(true);

// Create GUI for viewing/tweaking values
const gui = new GUI();
gui.close();

const baseFolder = gui.addFolder('Base');
let [fieldWidth, fieldHeight] = fieldConfig['field-size']
fieldWidth = feetToMeters(fieldWidth);
fieldHeight = feetToMeters(fieldHeight);
baseFolder.add(base.position, 'x', -fieldWidth / 2, fieldWidth / 2, 0.01).name('X-Pos');
baseFolder.add(base.position, 'z', -fieldHeight / 2, fieldHeight / 2, 0.01).name('Y-Pos');
baseFolder.add(base.rotation, 'y', -Math.PI, Math.PI, 0.01).name('Rotation');
baseFolder.open();

const turretFolder = gui.addFolder('Turret');
turretFolder.add(turret.rotation, 'y', degToRad(-180), degToRad(180), 0.01).name('Rotation');
turretFolder.open();

const joint1Folder = gui.addFolder('Joint1');
joint1Folder.add(joint1.rotation, 'z', degToRad(-180), degToRad(180), 0.01).name('Angle');
joint1Folder.open();

const joint2Folder = gui.addFolder('Joint2');
joint2Folder.add(joint2.rotation, 'z', degToRad(-180), degToRad(180), 0.01).name('Angle');
joint2Folder.open();

const joint3Folder = gui.addFolder('Joint3');
joint3Folder.add(joint3.rotation, 'z', degToRad(-180), degToRad(180), 0.01).name('Angle');
joint3Folder.open();

const wristFolder = gui.addFolder('Wrist');
wristFolder.add(wrist.rotation, 'y', degToRad(-180), degToRad(180), 0.01).name('Angle');
wristFolder.open();

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
        base.position.x = 0;
        base.position.z = 0;
        turret.rotation.y = 0;
        joint1.rotation.z = 0;
        joint2.rotation.z = 0;
        joint3.rotation.z = 0;
        wrist.rotation.y = 0;
        setClampOpen(true);
        updateGuiControllers();

        // Send to main/ntClient (example)
        window.electronAPI.set('resetCount', resetCount++);
    },
    toggleClamp: () => {
        setClampOpen(!clampOpen);
    },
};

const actionsFolder = gui.addFolder('Actions');
actionsFolder.add(gridHelper, 'visible').name('Toggle Grid');
actionsFolder.add(controlsGizmo.domElement, 'hidden').name('Hide Orbit Controls');
actionsFolder.add(controller, 'toggleClamp').name('Toggle Clamp');
actionsFolder.add(controller, 'reset').name('Reset');
actionsFolder.open();

// Configure interprocess hooks
window.electronAPI.onUpdate((_: any, key: string, value: any) => {
    // Ignore NaN / Null values
    if (value !== value || value === null) {
        return;
    }

    let handled = true;
    if (key === '/SmartDashboard/Arm/turretAngle') {
        turret.rotation.y = value;
    } else if (key === '/SmartDashboard/Arm/firstJointAngle') {
        joint1.rotation.z = -value;
    } else if (key === '/SmartDashboard/Arm/secondJointAngle') {
        joint2.rotation.z = -value;
    } else if (key === '/SmartDashboard/Arm/thirdJointAngle') {
        joint3.rotation.z = -value;
    } else if (key === '/SmartDashboard/Arm/wristAngle') {
        wrist.rotation.y = value;
    } else if (key === '/SmartDashboard/Arm/grabberClamp') {
        setClampOpen(value === 'Open');
    } else if (key === '/SmartDashboard/Drivetrain/x-pos') {
        base.position.x = value - (fieldWidth / 2);
    } else if (key === '/SmartDashboard/Drivetrain/y-pos') {
        base.position.z = (fieldHeight / 2) - value;
    } else if (key === '/SmartDashboard/Drivetrain/rot-degrees') {
        base.rotation.y = degToRad(value);
    } else {
        handled = false;
    }
    if (handled) {
        updateGuiControllers();
    }
    if (DEBUG) {
        console.log(`update: ${key} - ${value}`);
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

// Notify main process that renderer is ready
window.electronAPI.set('rendererReady', true);
