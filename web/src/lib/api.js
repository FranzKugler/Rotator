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
