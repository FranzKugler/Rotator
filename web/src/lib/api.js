export async function requestJson(url, fetcher = fetch) {
  const response = await fetcher(url);
  if (!response.ok) throw new Error((await response.text()) || `HTTP ${response.status}`);
  return response.json();
}

export async function postJson(url, body, fetcher = fetch) {
  const response = await fetcher(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body)
  });
  if (!response.ok) throw new Error((await response.text()) || `HTTP ${response.status}`);
  return response.headers.get('content-type')?.includes('application/json')
    ? response.json()
    : response.text();
}

export function fetchOtaStatus(fetcher = fetch) {
  return requestJson('/ota/status', fetcher);
}

export function checkForUpdate(fetcher = fetch) {
  return requestJson('/ota/check', fetcher);
}

export function installUpdate(fetcher = fetch) {
  return postJson('/ota/install', {}, fetcher);
}

export function setOtaConfig(config, fetcher = fetch) {
  return postJson('/ota/config', config, fetcher);
}

export function fetchLog(since = 0, fetcher = fetch) { return requestJson(`/log?since=${since}`, fetcher); }

export function fetchExpertStatus(fetcher = fetch) { return requestJson('/expert', fetcher); }
export function enrollExpert(password, fetcher = fetch) { return postJson('/expert/enroll', { password }, fetcher); }
export function unlockExpert(password, fetcher = fetch) { return postJson('/expert/unlock', { password }, fetcher); }
export function lockExpert(fetcher = fetch) { return postJson('/expert/lock', {}, fetcher); }
export function resetExpert(fetcher = fetch) { return postJson('/expert/reset', {}, fetcher); }

export function connectAngles(onAngle, locationLike = location, Socket = WebSocket) {
  const protocol = locationLike.protocol === 'https:' ? 'wss:' : 'ws:';
  const socket = new Socket(`${protocol}//${locationLike.host}/api/info/events`);
  socket.addEventListener('message', ({ data }) => onAngle(JSON.parse(data)));
  return socket;
}

export function calibrationStream(path, completeEvent, handlers, Source = EventSource) {
  const stream = new Source(path);
  stream.addEventListener('progress', ({ data }) => handlers.progress(Number(data)));
  stream.addEventListener(completeEvent, ({ data }) => {
    handlers.complete(data);
    stream.close();
  });
  stream.onerror = () => {
    handlers.error?.();
    stream.close();
  };
  return stream;
}

export function uploadImage(url, file, onProgress, Xhr = XMLHttpRequest) {
  return new Promise((resolve, reject) => {
    const xhr = new Xhr();
    xhr.open('POST', url, true);
    xhr.upload.onprogress = (event) => {
      if (event.lengthComputable) onProgress(Math.round((event.loaded * 100) / event.total));
    };
    xhr.onload = () => xhr.status === 200 ? resolve(xhr.responseText) : reject(new Error(xhr.responseText || `HTTP ${xhr.status}`));
    xhr.onerror = () => reject(new Error('Upload failed'));
    xhr.send(file);
  });
}

/* ------ the filesystem, for the Storage tab's explorer ------
 *
 * This is LittleFS, not NVS: the partition the web UI itself is served
 * from. These throw rather than going through postJson()'s bare-boolean
 * style - the explorer has a tree to refresh and a message to show next to
 * the file, and "something went wrong" is not that.
 */

export function fetchDirectory(path = '/', fetcher = fetch) {
  return requestJson(`/fs/list?path=${encodeURIComponent(path)}`, fetcher);
}

/** One file as text, for the editor. Binary files are downloaded instead. */
export async function fetchFile(path, fetcher = fetch) {
  const response = await fetcher(`/fs/read?path=${encodeURIComponent(path)}`);
  if (!response.ok) throw new Error((await response.text()) || `HTTP ${response.status}`);
  return response.text();
}

/**
 * Where to point a download link.
 *
 * A plain link rather than a fetch and a blob: expert mode is a flag the
 * firmware already checked once the page loaded, not a header this page
 * adds, so an ordinary GET is already authorised and the browser saves the
 * file itself with no copy in memory.
 */
export const fileUrl = (path) => `/fs/read?path=${encodeURIComponent(path)}&download=1`;

export const saveFile = (path, content, fetcher = fetch) => postJson('/fs/save', { path, content }, fetcher);
export const deleteEntry = (path, fetcher = fetch) => postJson('/fs/delete', { path }, fetcher);
export const makeDirectory = (path, fetcher = fetch) => postJson('/fs/mkdir', { path }, fetcher);

/**
 * Sends one file to the rotator as a raw body, so the firmware can stream
 * it straight to flash instead of holding it in the heap - the same shape
 * as installUpdate()'s OTA upload, not multipart: esp_http_server has no
 * multipart parser, and there is no reason to add one for this.
 */
export function uploadFile(path, file, onProgress, Xhr = XMLHttpRequest) {
  return new Promise((resolve, reject) => {
    const xhr = new Xhr();
    xhr.open('POST', `/fs/upload?path=${encodeURIComponent(path)}`, true);
    xhr.upload.onprogress = (event) => {
      if (event.lengthComputable) onProgress(event.loaded / event.total);
    };
    xhr.onload = () => {
      if (xhr.status >= 200 && xhr.status < 300) {
        try { resolve(JSON.parse(xhr.responseText)); }
        catch { resolve({}); }
        return;
      }
      reject(new Error(xhr.responseText || `HTTP ${xhr.status}`));
    };
    xhr.onerror = () => reject(new Error('Upload failed'));
    xhr.send(file);
  });
}

/* ------ NVS, shown as a two-level tree beside the filesystem ------
 *
 * A namespace is not a folder and a key is not a file - see NvsRoutes.h for
 * how far that pretence is carried and exactly where it stops. These four
 * calls take the namespace and key apart rather than a path, because that
 * is what the store actually has.
 */

export function fetchNvs(fetcher = fetch) {
  return requestJson('/nvs/list', fetcher);
}

/** One value as text. Blobs have none and are downloaded instead. */
export async function readNvs(ns, key, fetcher = fetch) {
  const response = await fetcher(nvsQuery(ns, key));
  if (!response.ok) throw new Error((await response.text()) || `HTTP ${response.status}`);
  return response.text();
}

export const nvsUrl = (ns, key) => `${nvsQuery(ns, key)}&download=1`;
export const saveNvs = (ns, key, content, fetcher = fetch) => postJson('/nvs/save', { ns, key, content }, fetcher);
export const deleteNvs = (ns, key, fetcher = fetch) => postJson('/nvs/delete', { ns, key }, fetcher);

const nvsQuery = (ns, key) => `/nvs/read?ns=${encodeURIComponent(ns)}&key=${encodeURIComponent(key)}`;

/**
 * Restarts the rotator.
 *
 * Its one caller is the NVS panel, where it is the answer to that panel's
 * own warning: some of the firmware's settings are cached in RAM and only
 * written back to NVS on their own save path, so an edit made straight into
 * NVS is only certain to apply after a restart.
 */
export function restartRotator(fetcher = fetch) {
  return postJson('/restart', {}, fetcher);
}
