// Service worker for the F350 dash.
//
// The original version was cache-first, which is why every update needed the
// site data cleared by hand before it would show up. This one is network-first
// for same-origin requests: online you always get the newest file, offline you
// fall back to the last good copy. Bumping CACHE_NAME still purges old caches.
const CACHE_NAME = 'f350-dash-v4';
const ASSETS = [
  './index.html',
  './app.js',
  './turbo.html',
  './scanner.html',
  './kaiser-turbo.jpg',
  './compressor-wheel.png',
  './manifest.json',
  './icon-192.png',
  './icon-512.png'
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    // Don't let one missing file abort the whole install.
    caches.open(CACHE_NAME).then((cache) =>
      Promise.all(ASSETS.map((a) => cache.add(a).catch(() => {})))
    )
  );
  self.skipWaiting();
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(keys.filter((k) => k !== CACHE_NAME).map((k) => caches.delete(k)))
    )
  );
  self.clients.claim();
});

self.addEventListener('fetch', (event) => {
  const req = event.request;
  if (req.method !== 'GET' || new URL(req.url).origin !== location.origin) return;

  event.respondWith(
    fetch(req)
      .then((res) => {
        // Refresh the cached copy in the background for offline use.
        const copy = res.clone();
        caches.open(CACHE_NAME).then((c) => c.put(req, copy)).catch(() => {});
        return res;
      })
      .catch(() => caches.match(req).then((c) => c || caches.match('./index.html')))
  );
});
