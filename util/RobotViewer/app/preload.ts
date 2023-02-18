import { contextBridge, ipcRenderer } from 'electron';

contextBridge.exposeInMainWorld('electronAPI', {
    set: (id: string, val: any) => ipcRenderer.invoke('set', id, val),
    onUpdate: (callback: any) => ipcRenderer.on('update', callback),
});
