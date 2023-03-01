import { app, BrowserWindow, ipcMain, ipcRenderer } from 'electron'
import path from 'path';
import { NetworkTables, NetworkTablesTypeInfos } from 'ntcore-ts-client';

const DEBUG = false;

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
    // const ntClient = NetworkTables.getInstanceByURI('127.0.0.1');
    const ntClient = NetworkTables.getInstanceByURI('10.54.68.2');

    await app.whenReady();
    const win = createWindow();

    const subscribe = () => {
        // TODO: don't brute force this
        const names = [
            '/SmartDashboard/Arm/turretAngle',
            '/SmartDashboard/Arm/firstJointAngle',
            '/SmartDashboard/Arm/secondJointAngle',
            '/SmartDashboard/Arm/thirdJointAngle',
            '/SmartDashboard/Arm/wristAngle',
            '/SmartDashboard/Arm/grabberClamp',
            '/SmartDashboard/Drivetrain/x-pos',
            '/SmartDashboard/Drivetrain/y-pos',
            '/SmartDashboard/Drivetrain/rot-degrees'
        ];

        names.forEach((name: string) => {
            // NOTE: default to subscribe as kDouble type; values of all types still come through
            const topic = ntClient.createTopic<number>(name, NetworkTablesTypeInfos.kDouble);
            topic.subscribe((val: number) => {
                win.webContents.send('update', name, val);
                if (DEBUG) {
                    console.log(`> ${name}: ${val}`);
                }
            }, true);
        });
    };

    // Bind IPC methods
    ipcMain.handle('set', (_, name: string, val: any) => {
        // TODO as needed (see: https://github.com/Chris2fourlaw/ntcore-ts-client/blob/main/README.md)
        //  ntClient.createTopic
        //  ntClient.publish
        //  ntClient.setValue
        // Wait until renderer is ready to subscribe and receive initial update
        if (name === 'rendererReady') {
            subscribe();
        }
        if (DEBUG) {
            console.log(`< ${name}: ${val}`);
        }
    });

    app.on('window-all-closed', () => {
        if (process.platform !== 'darwin') {
            app.quit();
        }
    });

    app.on('activate', () => {
        if (BrowserWindow.getAllWindows().length === 0) {
            createWindow();
        }
    });
}
main();
