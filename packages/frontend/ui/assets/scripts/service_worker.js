/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
10.08.26, 13:59

Service Worker for PWA support
*/

// For now, just a fake fetch handler that directly passes requests to the network
self.addEventListener('fetch', (event) => {
  event.respondWith(fetch(event.request));
});