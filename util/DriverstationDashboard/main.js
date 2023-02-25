const { app, BrowserWindow, ipcMain } = require('electron');
const EventEmitter = require('events');
const path = require('path');
const { NetworkTables, NetworkTablesTypeInfos } = require('ntcore-ts-client');
const ntcore = NetworkTables.getInstanceByURI("127.0.0.1");

if (require('electron-squirrel-startup')) app.quit();

const stationSelectorTopic = ntcore.createTopic('/customDS/location', NetworkTablesTypeInfos.kString);
const indicatorTopic = ntcore.createTopic('/customDS/indicator', NetworkTablesTypeInfos.kString);

async function main() {

    const createWindow = () => {
        const win = new BrowserWindow({
            width: 1920,
            height: 420,
            webPreferences: {
                preload: path.join(__dirname, 'preload.js'),
            }
        })
        ipcMain.handle('setStation', (_, val) => stationSelectorTopic.setValue(val));
        ipcMain.handle('setIndicator', (_, val) => indicatorTopic.setValue(val));
        ipcMain.handle('ready', () => ntcore.isRobotConnected());
        ipcMain.handle('publish', () => {
            stationSelectorTopic.announce();
            stationSelectorTopic.publish();
            indicatorTopic.announce();
            indicatorTopic.publish();
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