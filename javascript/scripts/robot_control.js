
// imports
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
// import * as SWS from 'simplewebserial';

// PARAMETERS
const Lx = 2150; const Ly = 2150; const Lz = 2500; // robot frame
const zARlow = 400; const zARhigh = 2180; const D_pulley = 85;// cable pulley elevations & diameter
const eLx = 230; const eLy = 230; const eLz = 230; // effector frame
const x0_e = 0; const y0_e = -Ly / 2 + eLy / 2; const z0_e = (zARhigh + zARlow) / 2; // effector frame
const stepsByMM = 16 * 200 * (60 / 30) / (Math.PI * 60); // microstep*(steps_by_turn)*(rollerpulley/motorpulley)/(mm_by_turn) motor steps by cable dL in millimeters
const robotCableAnchorPositions = [
    Lx / 2, -Ly / 2, zARlow - D_pulley / 2,
    Lx / 2, -Ly / 2, zARhigh + D_pulley / 2,
    -Lx / 2, -Ly / 2, zARlow - D_pulley / 2,
    -Lx / 2, -Ly / 2, zARhigh + D_pulley / 2,
    Lx / 2, Ly / 2, zARlow - D_pulley / 2,
    Lx / 2, Ly / 2, zARhigh + D_pulley / 2,
    -Lx / 2, Ly / 2, zARlow - D_pulley / 2,
    -Lx / 2, Ly / 2, zARhigh + D_pulley / 2,
]
const effectorCableAnchorPositions = [
    eLx / 2, -eLy / 2, -eLz / 2,
    eLx / 2, -eLy / 2, eLz / 2,
    -eLx / 2, -eLy / 2, -eLz / 2,
    -eLx / 2, -eLy / 2, eLz / 2,
    eLx / 2, eLy / 2, -eLz / 2,
    eLx / 2, eLy / 2, eLz / 2,
    -eLx / 2, eLy / 2, -eLz / 2,
    -eLx / 2, eLy / 2, eLz / 2,
]

// THREE renderer window
const renderer = new THREE.WebGLRenderer({
    antialias: true,
});
const container = document.getElementById('scene-view'); // ou ta div parent
renderer.setSize(container.clientWidth, container.clientHeight);
renderer.setClearColor(0xffffff, 0);
document.getElementById('scene-view').appendChild(renderer.domElement);

// THREE scene
const scene = new THREE.Scene();
const axesHelper = new THREE.AxesHelper(100);
axesHelper.setColors(0xff0000, 0x00ff00, 0x0000ff);
scene.add(axesHelper);

// THREE camera
const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, .001, 9999999);
camera.position.set(-Lx, -1.75 * Ly, .75 * Lz);
camera.up.set(0, 0, 1);
camera.aspect = container.clientWidth / container.clientHeight;
camera.updateProjectionMatrix();

const orbit = new OrbitControls(camera, renderer.domElement);
orbit.target.set(0, 0, Lz / 2); // camera.lookAt

// ROBOT FRAME
const robotLineMaterial = new THREE.LineBasicMaterial({
    color: 0x000000,
});
const robotGeometry = new THREE.BufferGeometry()
    .setAttribute('position', new THREE.Float32BufferAttribute([
        -Lx / 2, -Ly / 2, 0,
        Lx / 2, -Ly / 2, 0,
        Lx / 2, Ly / 2, 0,
        -Lx / 2, Ly / 2, 0,
        -Lx / 2, -Ly / 2, Lz,
        Lx / 2, -Ly / 2, Lz,
        Lx / 2, Ly / 2, Lz,
        -Lx / 2, Ly / 2, Lz,
    ], 3))
    .setIndex([
        0, 4,
        1, 5,
        2, 6,
        3, 7,
        4, 5,
        5, 6,
        6, 7,
        7, 4,
    ]);
const robotObject = new THREE.LineSegments(robotGeometry, robotLineMaterial);
scene.add(robotObject);

// CABLE ANCHORS ON THE FIXED ROBOT FRAME
const robotCableAnchorGeometry = new THREE.BufferGeometry()
    .setAttribute('position', new THREE.Float32BufferAttribute(robotCableAnchorPositions, 3))
const robotCableAnchorMaterial = new THREE.PointsMaterial({
    color: 0x000000,
    size: 50,
});
const robotCableAnchors = new THREE.Points(robotCableAnchorGeometry, robotCableAnchorMaterial);
scene.add(robotCableAnchors);


// CABLE ANCHORS ON THE MOBILE EFFECTOR
const effectorCableAnchorGeometry = new THREE.BufferGeometry()
    .setAttribute('position', new THREE.Float32BufferAttribute(effectorCableAnchorPositions, 3))
const effectorCableAnchorMaterial = new THREE.PointsMaterial({
    color: 0xff0000,
    size: 50,
});
const effectorCableAnchors = new THREE.Points(effectorCableAnchorGeometry, effectorCableAnchorMaterial);
scene.add(effectorCableAnchors);

// EFFECTOR
const effectorLineMaterial = new THREE.LineBasicMaterial({
    color: 0xff0000,
});
const effectorLineGeometry = new THREE.BufferGeometry()
    .setAttribute('position', new THREE.Float32BufferAttribute([
        -eLx / 2, -eLy / 2, 0,
        eLx / 2, -eLy / 2, 0,
        eLx / 2, eLy / 2, 0,
        -eLx / 2, eLy / 2, 0,
    ], 3))
    .setIndex([
        0, 1,
        1, 2,
        2, 3,
        3, 0,
    ]);
const effectorLines = new THREE.LineSegments(effectorLineGeometry, effectorLineMaterial);
const effector = new THREE.Group().add(effectorCableAnchors).add(effectorLines)
scene.add(effector);

// EFFECTOR CONTROL
const control = new TransformControls(camera, renderer.domElement);
control.addEventListener('dragging-changed', function (event) {
    orbit.enabled = !event.value;
});
control.attach(effector);
const gizmo = control.getHelper();
scene.add(gizmo);
control.setSpace('local');




// INITIAL EFFECTOR POSITION
// effector.rotation.x = Math.PI/2
effector.position.set(x0_e, y0_e, z0_e)

function updateEffectorFromPosition() {
    effector.position.set(position.x, position.y, position.z);
    updateCablePositions();
    updateCableLengths();
    updateMotorPositions();
    updatePositionInputs();
}


// CABLES
const cableLineMaterial = new THREE.LineBasicMaterial({
    color: 0x0000ff,
});
const cablePositions = new THREE.Float32BufferAttribute(robotCableAnchorPositions.concat(effectorCableAnchorPositions), 3);
const cableGeometry = new THREE.BufferGeometry()
    .setAttribute('position', cablePositions)
    .setIndex([
        0, 8,
        1, 9,
        2, 10,
        3, 11,
        4, 12,
        5, 13,
        6, 14,
        7, 15,
    ]);
const cables = new THREE.LineSegments(cableGeometry, cableLineMaterial);
scene.add(cables);

function updateCablePositions() {
    for (let cable = 0; cable < 8; cable++) {
        let pos = new THREE.Vector3(
            effectorCableAnchorPositions[3 * cable],
            effectorCableAnchorPositions[3 * cable + 1],
            effectorCableAnchorPositions[3 * cable + 2]
        );
        pos = effector.localToWorld(pos)
        cablePositions.setXYZ(cable + 8, pos.x, pos.y, pos.z);
    }
    cableGeometry.attributes.position.needsUpdate = true;
}
updateCablePositions();

const cableLengths = [0, 0, 0, 0, 0, 0, 0, 0];
function updateCableLengths() {
    for (let cable = 0; cable < 8; cable++) {
        let vec = new THREE.Vector3(
            cablePositions.getX(cable) - cablePositions.getX(cable + 8),
            cablePositions.getY(cable) - cablePositions.getY(cable + 8),
            cablePositions.getZ(cable) - cablePositions.getZ(cable + 8),
        );
        cableLengths[cable] = vec.length();
    }
    // console.log(cableLengths)
}
updateCableLengths();

const motorPositions = [0, 0, 0, 0, 0, 0, 0, 0];
function updateMotorPositions() {
    for (let mot = 0; mot < 8; mot++) {
        motorPositions[mot] = Math.round(stepsByMM * cableLengths[mot]);
    }
    const stepsDisplay = document.getElementById('motorSteps');
    stepsDisplay.textContent = motorPositions.join(', ');

}
updateMotorPositions();


// KEYBOARD CALLBACKS
window.addEventListener('keydown', function (event) {
    switch (event.key) {
        case 'r': control.setMode('rotate'); break;
        case 't': control.setMode('translate'); break;
    }
});






// ROBOT CONTROL ==================
const position = { x: 0, y: 0, z: 0 };
let step = 10;

// Bind elements
const xInput = document.getElementById('xInput');
const yInput = document.getElementById('yInput');
const zInput = document.getElementById('zInput');

const stepSlider = document.getElementById('stepSlider');
const stepValue = document.getElementById('stepValue');

const buttons = document.querySelectorAll('.direction-controls button');

// Update inputs when position changes
function updatePositionInputs() {
    xInput.value = position.x;
    yInput.value = position.y;
    zInput.value = position.z;
}


// HOMING FUNCTION
$("#homing-btn").on("click", function () {
    xInput.value = 0;
    yInput.value = 0;
    zInput.value = 0;
    onManualInputChange()
});

// CENTER POSITIONING FUNCTION
$("#center-btn").on("click", function () {
    xInput.value = 0;
    yInput.value = 0;
    zInput.value = Lz / 2 + 40;
    onManualInputChange()
});
// BORDER POSITIONING FUNCTION
$("#border-btn").on("click", function () {
    xInput.value = -960;
    yInput.value = -960;
    zInput.value = zARlow - D_pulley / 2;
    onManualInputChange()
});

// Apply move based on direction
function moveRobot(direction) {
    switch (direction) {
        case 'haut': position.z += step; break;
        case 'bas': position.z -= step; break;
        case 'gauche': position.x -= step; break;
        case 'droite': position.x += step; break;
        case 'avant': position.y += step; break;
        case 'arrière': position.y -= step; break;
    }
    console.log(`Moved ${direction}, new position:`, position);
    updatePositionInputs();
    updateEffectorFromPosition();
}

// Synchronise slider <-> pas
stepSlider.addEventListener('input', () => {
    step = parseInt(stepSlider.value);
    stepValue.textContent = step;
});

// Écoute des boutons directionnels
buttons.forEach(btn => {
    btn.addEventListener('click', () => {
        const dir = btn.getAttribute('data-dir');
        moveRobot(dir);
    });
});

// Mise à jour manuelle des inputs
function onManualInputChange() {
    position.x = parseFloat(xInput.value) || 0;
    position.y = parseFloat(yInput.value) || 0;
    position.z = parseFloat(zInput.value) || 0;
    updateEffectorFromPosition();
}

xInput.addEventListener('change', onManualInputChange);
yInput.addEventListener('change', onManualInputChange);
zInput.addEventListener('change', onManualInputChange);


// Init
position.x = x0_e;
position.y = y0_e;
position.z = z0_e;
updateEffectorFromPosition(); // met à jour visuel dès le départ
updatePositionInputs(); // synchro champs
stepValue.textContent = step;






// CHECK 3D VIEW MODIFICATION
control.addEventListener('objectChange', () => {
    // Met à jour l'objet position JS
    position.x = effector.position.x;
    position.y = effector.position.y;
    position.z = effector.position.z;

    // Met à jour les champs input
    updatePositionInputs();

    // Met à jour les longueurs de câbles
    updateCablePositions();
    updateCableLengths();
    updateMotorPositions();
});



// CONTROL CONFIGURATION ==================
const config = {
    sendPositions: true,
    acceleration: 10000,
    maxspeed: 2000,
};

// Bind elements
const checkbox = document.getElementById('sendPositions');

const accelSlider = document.getElementById('acceleration');
const accelInput = document.getElementById('acceleration-input');

const maxSlider = document.getElementById('maxspeed');
const maxInput = document.getElementById('maxspeed-input');

// Checkbox binding
checkbox.addEventListener('change', () => {
    config.sendPositions = checkbox.checked;
});

// Sync Acceleration
accelSlider.addEventListener('input', () => {
    const value = parseInt(accelSlider.value);
    config.acceleration = value;
    accelInput.value = value;
    console.log('Acceleration set to:', value);
});

accelInput.addEventListener('input', () => {
    let value = parseInt(accelInput.value);
    if (isNaN(value)) return;
    value = Math.min(Math.max(value, 100), 20000); // Clamp
    config.acceleration = value;
    accelSlider.value = value;
    console.log('Acceleration input set to:', value);
});

// Sync Maxspeed
maxSlider.addEventListener('input', () => {
    const value = parseInt(maxSlider.value);
    config.maxspeed = value;
    maxInput.value = value;
});

maxInput.addEventListener('input', () => {
    let value = parseInt(maxInput.value);
    if (isNaN(value)) return;
    value = Math.min(Math.max(value, 100), 10000);
    config.maxspeed = value;
    maxSlider.value = value;
});




// SERIAL COMMUNICATION
const connection = SimpleWebSerial.setupSerialConnection({
    baudRate: 250000,
    requestAccessOnPageLoad: true,
    logIncomingSerialData: false,
    logOutgoingSerialData: false,
});
connection.on('hexadecimal', (value) => { console.log(value) });
connection.on("data", function (data) {
    console.log(data);
})

var lastSentMotorPositions = motorPositions.slice();
var lastSend = performance.now();
async function sendMotorPositions() {
    // console.log(cableLengths) ;
    let samePos = lastSentMotorPositions.every((v, i) => v === motorPositions[i]);
    if (samePos) {
        // console.log("Motor positions did not change");
        return;
    }
    // console.log(motorPositions);
    // console.log(lastSentMotorPositions);
    const r = motorPositions[0] - lastSentMotorPositions[0];
    const g = motorPositions[1] - lastSentMotorPositions[1];
    const b = motorPositions[2] - lastSentMotorPositions[2];
    const a = motorPositions[3] - lastSentMotorPositions[3];
    let data = { r, g, b, a }

    console.log(data);
    // Affichage données dans console
    $('#motorPositions').text(
        Object.entries(data).map(([key, val]) => `${key}: ${val}`).join(", ")
    );


    await connection.send("move", data);


    lastSentMotorPositions = motorPositions.slice();

    // console.log(lastSentMotorPositions)
}
// ANIMATION FUNCTION
async function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);

    // Follow window size changes
    const container = document.getElementById('scene-view');
    renderer.setSize(container.clientWidth, container.clientHeight);
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();

    renderer.render(scene, camera);

    camera.updateProjectionMatrix();
    orbit.update();

    // Update cables
    updateCablePositions();
    updateCableLengths();
    updateMotorPositions();
    if (config.sendPositions && (performance.now() - lastSend) > 500) {
        await sendMotorPositions();
        lastSend = performance.now();
    }

    renderer.render(scene, camera);
};

animate();