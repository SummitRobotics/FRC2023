const { app, BrowserWindow, ipcMain } = require('electron');
const EventEmitter = require('events');
const path = require('path');
const ntClient = require('wpilib-nt-client');
const client = new ntClient.Client()

const retryEvent = new EventEmitter();
let connected = false;

if (require('electron-squirrel-startup')) app.quit();

retryEvent.on('startFailed', async () => {
    await new Promise(r => setTimeout(r, 1000));
    client.start((isConnected, err) => {
        console.log({ isConnected, err });
        connected = isConnected;
        if (!connected) {
            retryEvent.emit('startFailed');
        }
    }, '10.54.68.2');
});

async function main () {
    client.start((isConnected, err) => {
        console.log({ isConnected, err });
        connected = isConnected;
        if (!connected) {
            retryEvent.emit('startFailed');
        }
    }, '10.54.68.2');
    
    client.setReconnectDelay(1000);

    const createWindow = () => {
        const win = new BrowserWindow({
            width: 1920,
            height: 420,
            webPreferences: {
                preload: path.join(__dirname, 'preload.js'),
            }
        })
        ipcMain.handle('assign', (_, id, val) => client.Assign(val, id, false));
        ipcMain.handle('ready', () => connected);
        win.loadFile('./html/index.html');
    }
    
    app.whenReady().then(() => {
    createWindow()
    })
    
    app.on('window-all-closed', () => {
        if (process.platform !== 'darwin') app.quit()
    })
    
    app.on('activate', () => {
        if (BrowserWindow.getAllWindows().length === 0) createWindow()
    })
}

main();