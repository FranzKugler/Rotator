import { describe, expect, it, vi } from 'vitest';
import {
  checkForUpdate,
  connectAngles,
  enrollExpert,
  fetchExpertStatus,
  fetchOtaStatus,
  installUpdate,
  lockExpert,
  postJson,
  setOtaConfig,
  unlockExpert,
  uploadImage
} from '../web/src/lib/api.js';

describe('postJson', () => {
  it('posts JSON and returns a JSON response', async () => {
    const fetcher = vi.fn().mockResolvedValue({
      ok: true,
      headers: { get: () => 'application/json' },
      json: async () => ({ connected: true })
    });

    await expect(postJson('/api/wifi/connect', { ssid: 'sky' }, fetcher))
      .resolves.toEqual({ connected: true });
    expect(fetcher).toHaveBeenCalledWith('/api/wifi/connect', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ ssid: 'sky' })
    });
  });

  it('surfaces the response text when a request fails', async () => {
    const fetcher = vi.fn().mockResolvedValue({ ok: false, text: async () => 'bad network' });
    await expect(postJson('/api/network/set/ip', {}, fetcher)).rejects.toThrow('bad network');
  });
});

describe('connectAngles', () => {
  it('uses wss on https pages and parses angle messages', () => {
    let socket;
    class FakeSocket {
      constructor(url) { this.url = url; this.listeners = {}; socket = this; }
      addEventListener(type, callback) { this.listeners[type] = callback; }
      close() {}
    }
    const onAngle = vi.fn();
    connectAngles(onAngle, { protocol: 'https:', host: 'rotator.local' }, FakeSocket);
    expect(socket.url).toBe('wss://rotator.local/api/info/events');
    socket.listeners.message({ data: '{"angle":12.5,"mechAngle":10,"direction":"cw"}' });
    expect(onAngle).toHaveBeenCalledWith({ angle: 12.5, mechAngle: 10, direction: 'cw' });
  });
});

describe('uploadImage', () => {
  it('sends the raw file and reports progress', async () => {
    let xhr;
    class FakeXHR {
      constructor() { this.upload = {}; xhr = this; }
      open(method, url) { this.method = method; this.url = url; }
      send(body) {
        this.body = body;
        this.upload.onprogress({ lengthComputable: true, loaded: 1, total: 2 });
        this.status = 200;
        this.onload();
      }
    }
    const progress = vi.fn();
    await uploadImage('/ota/upload', 'binary', progress, FakeXHR);
    expect([xhr.method, xhr.url, xhr.body]).toEqual(['POST', '/ota/upload', 'binary']);
    expect(progress).toHaveBeenCalledWith(50);
  });
});

describe('OTA release channel API', () => {
  it('loads status and checks the selected GitHub channel', async () => {
    const fetcher = vi.fn()
      .mockResolvedValueOnce({ ok: true, json: async () => ({ firmwareVersion: '0.8.0' }) })
      .mockResolvedValueOnce({ ok: true, json: async () => ({ availableVersion: '0.9.0' }) });

    await expect(fetchOtaStatus(fetcher)).resolves.toEqual({ firmwareVersion: '0.8.0' });
    await expect(checkForUpdate(fetcher)).resolves.toEqual({ availableVersion: '0.9.0' });
    expect(fetcher).toHaveBeenNthCalledWith(1, '/ota/status');
    expect(fetcher).toHaveBeenNthCalledWith(2, '/ota/check');
  });

  it('installs an update and persists channel configuration', async () => {
    const fetcher = vi.fn().mockResolvedValue({
      ok: true,
      headers: { get: () => 'application/json' },
      json: async () => ({ state: 'downloading' })
    });

    await installUpdate(fetcher);
    await setOtaConfig({ channel: 1, autoUpdate: true, checkInterval: 24 }, fetcher);
    expect(fetcher).toHaveBeenNthCalledWith(1, '/ota/install', {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}'
    });
    expect(fetcher).toHaveBeenNthCalledWith(2, '/ota/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ channel: 1, autoUpdate: true, checkInterval: 24 })
    });
  });
});

describe('expert update lock API', () => {
  it('loads, enrolls, unlocks and locks expert mode', async () => {
    const fetcher = vi.fn().mockResolvedValue({
      ok: true,
      headers: { get: () => 'application/json' },
      json: async () => ({ unlocked: true })
    });
    await fetchExpertStatus(fetcher);
    await enrollExpert('secret12', fetcher);
    await unlockExpert('secret12', fetcher);
    await lockExpert(fetcher);
    expect(fetcher.mock.calls.map((call) => call[0])).toEqual([
      '/expert', '/expert/enroll', '/expert/unlock', '/expert/lock'
    ]);
  });
});
