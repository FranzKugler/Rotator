import { describe, expect, it, vi } from 'vitest';
import { connectAngles, postJson, uploadImage } from '../web/src/lib/api.js';

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
    await uploadImage('/update_raw', 'binary', progress, FakeXHR);
    expect([xhr.method, xhr.url, xhr.body]).toEqual(['POST', '/update_raw', 'binary']);
    expect(progress).toHaveBeenCalledWith(50);
  });
});
