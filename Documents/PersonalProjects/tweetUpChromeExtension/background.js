console.log('Hello from background.js')
chrome.runtime.onInstalled.addListener(() => {
  chrome.tabs.query({ active: true, currentWindow: true }, (tabs) => {
    const url = new URL(chrome.runtime.getURL(""));
    console.log(url.href);
  });
});
