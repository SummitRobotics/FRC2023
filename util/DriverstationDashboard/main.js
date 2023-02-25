const { app, BrowserWindow, ipcMain } = require('electron');
const EventEmitter = require('events');
const path = require('path');
const { NetworkTables, NetworkTablesTypeInfos } = require('ntcore-ts-client');
const ntcore = NetworkTables.getInstanceByURI("127.0.0.1");

if (require('electron-squirrel-startup')) app.quit();

const stationSelectorTopic = ntcore.createTopic('/stationSelector', NetworkTablesTypeInfos.kString);

async function main() {

    const createWindow = () => {
        const win = new BrowserWindow({
            width: 1920,
            height: 420,
            webPreferences: {
                preload: path.join(__dirname, 'preload.js'),
            }
        })
        ipcMain.handle('assign', (_, id, val) => stationSelectorTopic.setValue(val));
        ipcMain.handle('ready', () => ntcore.isRobotConnected());
        ipcMain.handle('publisher', () => {
            stationSelectorTopic.announce();
            stationSelectorTopic.publish();
        })
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