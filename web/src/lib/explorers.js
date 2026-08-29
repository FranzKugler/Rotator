/**
 * explorers
 * The two things the file explorer can be pointed at, behind one interface.
 *
 * LittleFS is a filesystem and NVS is a key-value store, and they are shown
 * the same way on purpose: namespaces read as folders, keys as files, and
 * the gestures are identical. One component draws both, so there is one
 * place a bug in the tree can live.
 *
 * The interface is deliberately narrow, and the differences it does admit
 * are the ones that would be lies to hide:
 *
 *   list(path)     one folder, as { entries, truncated, used, total }
 *   readText(path) the content, for the editor
 *   urlOf(path)    where a download link points
 *   save/remove    the writes both stores have
 *   upload/mkdir   present only where they mean something
 *   unit           'bytes' or 'entries', because NVS counts in entries
 *
 * The one structural difference is how a listing arrives. LittleFS answers
 * per directory and the tree expands lazily, which is what keeps a full
 * directory from making every response slow. NVS has no such thing: the
 * iterator walks the whole partition regardless, so it is fetched once and
 * sliced here. Hiding that behind list() means the component never has to
 * know.
 */
import {
  fetchDirectory, fetchFile, fileUrl, saveFile, deleteEntry, makeDirectory, uploadFile,
  fetchNvs, readNvs, nvsUrl, saveNvs, deleteNvs
} from './api.js';

/** The rotator's LittleFS: the partition this page itself is served from. */
export function littleFs() {
  return {
    key: 'littlefs',
    unit: 'bytes',
    canUpload: true,
    canMkdir: true,

    async list(path) {
      const answer = await fetchDirectory(path);
      return {
        entries: answer.entries,
        truncated: !!answer.truncated,
        used: answer.used,
        total: answer.total
      };
    },

    readText: (path) => fetchFile(path),
    urlOf: (path) => fileUrl(path),
    save: (path, content) => saveFile(path, content),
    remove: (path) => deleteEntry(path),
    mkdir: (path) => makeDirectory(path),
    upload: (path, file, onProgress) => uploadFile(path, file, onProgress)
  };
}

/**
 * NVS, read as a two-level tree.
 *
 * `/` lists the namespaces, `/<namespace>` lists its keys, and there is no
 * third level - not because it is forbidden but because NVS has no such
 * thing. A path here is `/namespace/key` and nothing else, so splitting it
 * is unambiguous.
 */
export function nvs() {
  // The whole partition, as last fetched. Refreshed whenever a listing is
  // asked for at the root, which is what every write triggers.
  let snapshot = null;

  const split = (path) => {
    const parts = path.split('/').filter(Boolean);
    return { ns: parts[0], key: parts[1] };
  };

  async function refresh() {
    snapshot = await fetchNvs();
    return snapshot;
  }

  return {
    key: 'nvs',
    unit: 'entries',
    canUpload: false,
    canMkdir: false,

    async list(path) {
      // The root is where a refresh happens, because that is the one
      // listing the component reloads after every write.
      const data = path === '/' || !snapshot ? await refresh() : snapshot;
      const volume = { used: data.used, total: data.total, truncated: !!data.truncated };

      if (path === '/') {
        // A namespace is a folder that exists because keys are in it; its
        // size is how many, which is the only honest number available.
        const names = [...new Set(data.entries.map((e) => e.ns))];
        return {
          ...volume,
          entries: names.map((name) => ({
            name,
            dir: true,
            size: data.entries.filter((e) => e.ns === name).length
          }))
        };
      }

      const { ns } = split(path);
      return {
        ...volume,
        truncated: false,
        entries: data.entries
          .filter((entry) => entry.ns === ns)
          .map((entry) => ({
            // The suffix is the firmware's reading of the value, not a name
            // it stored - see main/NvsRoutes.c.
            name: `${entry.key}.${entry.suffix}`,
            dir: false,
            size: entry.size,
            edit: !!entry.edit,
            protected: !!entry.protected,
            // Kept so the writes do not have to undo the suffix again.
            nvsKey: entry.key,
            nvsType: entry.type
          }))
      };
    },

    /** Paths carry the invented suffix; the store never saw it. */
    readText(path) {
      const { ns, key } = split(path);
      return readNvs(ns, stripSuffix(key));
    },

    urlOf(path) {
      const { ns, key } = split(path);
      return nvsUrl(ns, stripSuffix(key));
    },

    async save(path, content) {
      const { ns, key } = split(path);
      const answer = await saveNvs(ns, stripSuffix(key), content);
      snapshot = null;
      return answer;
    },

    async remove(path) {
      const { ns, key } = split(path);
      // A namespace is not a thing that can be deleted; it goes when its
      // last key does. Asking to delete a folder here is a mistake, not an
      // action.
      if (key === undefined) throw new Error('A namespace cannot be deleted directly.');
      const answer = await deleteNvs(ns, stripSuffix(key));
      snapshot = null;
      return answer;
    }
  };
}

/** `conf.json` back to `conf`. Only the last dot, so a key may contain one. */
function stripSuffix(name) {
  const cut = name.lastIndexOf('.');
  return cut > 0 ? name.substring(0, cut) : name;
}
