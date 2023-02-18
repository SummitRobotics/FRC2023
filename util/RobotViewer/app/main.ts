import { app, BrowserWindow } from 'electron'

const createWindow = () => {
    const win = new BrowserWindow({
        width: 800,
        height: 600
    });

    win.loadFile('./app/index.html');
}

app.whenReady().then(() => {
    createWindow();
});
