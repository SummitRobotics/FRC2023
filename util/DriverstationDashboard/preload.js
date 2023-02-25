const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('ntClient', {
  setStation: (val) => ipcRenderer.invoke('setStation', val),
  setIndicator: (val) => ipcRenderer.invoke('setIndicator', val),
  isReady: () => ipcRenderer.invoke('ready'),
  publish: () => ipcRenderer.invoke('publish')
})