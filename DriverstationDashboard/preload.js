const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('ntClient', {
  set: (id, val) => ipcRenderer.invoke('assign', id, val),
  isReady: () => ipcRenderer.invoke('ready'),
})