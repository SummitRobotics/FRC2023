function generateCubeHtml(row, col) {
    return `<div class='item cube' id='id${row}${col}'></div>`
}
function generateConeHtml(row, col) {
    return `<div class='item cone' id='id${row}${col}'></div>`
}
function generateAnyHtml(row, col) {
    return `<div class='item any' id='id${row}${col}'></div>`
}

function updateSelected(row, col) {
    console.log('here');
    const allPrevSelected = document.getElementsByClassName('selected item');
    for (const prevSelected of allPrevSelected) {
        prevSelected.classList.toggle('selected');
    }

    const element = document.getElementById(`id${row}${col}`);
    if (element) {
        element.classList.toggle('selected');
    }

    window.ntClient.setStation(`${row}${col}`);
}

function setSelector(type) {
    const allPrevSelected = document.getElementsByClassName('selected button');
    for (const prevSelected of allPrevSelected) {
        prevSelected.classList.toggle('selected');
    }

    const element = document.getElementById(type);
    if (element) {
        element.classList.toggle('selected');
    }

    console.log(type);
    window.ntClient.setIndicator(type);
}

async function main() {
    // Wait for Network tables to be connected
    while (!await window.ntClient.isReady()) {
        await new Promise(r => setTimeout(r, 1000));
    }

    window.ntClient.publish();

    const buttonsDiv = document.getElementById('buttons');
    let innerHtml = '';

    for (let row = 1; row <= 3; row++) {
        for (let col = 1; col <= 9; col++) {
            if (row == 1) {
                innerHtml += generateAnyHtml(row, col);
            } else if (col === 2 || col === 5 || col === 8) {
                innerHtml += generateCubeHtml(row, col);
            } else {
                innerHtml += generateConeHtml(row, col);
            }
        }
    }
    buttonsDiv.innerHTML = innerHtml;

    const selectorDiv = document.getElementById('signaler');
    selectorDiv.innerHTML = '<div class="button cone" id="cone"></div><div class="button any" id="none"></div><div class="button cube" id="cube"></div>';

    const buttons = document.getElementsByClassName('item');

    for (const button of buttons) {
        button.addEventListener('click', () => updateSelected(button.id.at(2), button.id.at(3)));
    }
    updateSelected(0, 0);

    const selectorButtons = document.getElementsByClassName('button');
    for (const button of selectorButtons) {
        button.addEventListener('click', () => setSelector(button.id));
    }
    setSelector('none');
}

main();