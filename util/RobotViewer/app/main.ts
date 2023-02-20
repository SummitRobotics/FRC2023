import { app, BrowserWindow, ipcMain, ipcRenderer } from 'electron'
import { Client } from 'wpilib-nt-client';
import path from 'path';

/**
 * Constructs a networktable client and connects to remote server.
 *
 * @param address The address of the networktable server.
 * @param connectDelay Delay before attempting to reconnected (ms).
 * @returns networktable client
 */
const createNtClient = (address: string, connectDelay: number = 1000): Client => {
    const client = new Client();
    client.setReconnectDelay(connectDelay);
    let hasConnected = false; // Client doesn't perform a reconnect until connected
    const connectHandler = (connected: boolean, err: Error) => {
        hasConnected = hasConnected || connected;
        console.log({ connected, err });
        if (!hasConnected) {
            setTimeout(() => {
                client.start(connectHandler, address);
            }, connectDelay);
        }
    };
    client.start(connectHandler, address);
    return client;
}

const createWindow = (): BrowserWindow => {
    const win = new BrowserWindow({
        width: 800,
        height: 600,
        webPreferences: {
            preload: path.join(__dirname, 'preload.js'),
        }
    });
    win.loadFile('./app/index.html');
    return win;
}

async function main() {
    // const ntClient = createNtClient('127.0.0.1');
    const ntClient = createNtClient('10.54.68.2');

    await app.whenReady();
    const win = createWindow();

    // Bind IPC methods
    ipcMain.handle('set', (_, name: string, val: any) => {
        console.log(`set: ${name} - ${val}`);
        ntClient.Assign(val, name);
    });
    ntClient.addListener((key: string, value: any, valueType: string, type: string, id: number, flags: number) => {
        win.webContents.send('update', key, value, valueType, type, id, flags);
        //console.log(`update: ${key} - ${value} - ${valueType} - ${type} - ${id} - ${flags}`);
    });

    app.on('window-all-closed', () => {
        if (process.platform !== 'darwin') {
            app.quit();
        }
    })

    app.on('activate', () => {
        if (BrowserWindow.getAllWindows().length === 0) {
            createWindow();
        }
    });
}
main();
