<script>
  /**
   * Storage
   * The rotator's two persistent stores, side by side.
   *
   * They are genuinely different things - one is a filesystem, the other a
   * key-value store - and they are shown the same way because the question
   * somebody arrives with is the same for both: what is in there, and can I
   * get it out. Naming them by what they are rather than by a metaphor is
   * deliberate; "LittleFS" and "NVS" are the words in every ESP32 document and
   * in this project's own logs, so somebody searching for why their settings
   * vanished finds the tab that holds them.
   *
   * What each one is gets a line of its own below the switch, because the
   * difference matters the moment anything is edited: LittleFS is wiped whole
   * by a filesystem update, NVS survives one. That is the whole reason the
   * WiFi credentials and the expert password live where they do.
   */
  import Explorer from './Explorer.svelte';
  import { littleFs, nvs } from '../lib/explorers.js';
  import { restartRotator } from '../lib/api.js';

  /* Built once. The NVS adapter caches a snapshot of the whole partition, so
     rebuilding it on every render would throw that away on each keystroke. */
  const stores = { littlefs: littleFs(), nvs: nvs() };

  let which = $state('littlefs');
</script>

<section class="card">
  <h2>Storage</h2>

  <div class="store-tabs" role="tablist" aria-label="Storage">
    <button type="button" role="tab" class:on={which === 'littlefs'}
            aria-selected={which === 'littlefs'} onclick={() => (which = 'littlefs')}>
      LittleFS
    </button>
    <button type="button" role="tab" class:on={which === 'nvs'}
            aria-selected={which === 'nvs'} onclick={() => (which = 'nvs')}>
      NVS
    </button>
  </div>

  {#if which === 'littlefs'}
    <Explorer store={stores.littlefs}
              hint="The rotator's filesystem — the same partition this page is served from. A web UI update overwrites it whole."
              warning="Delete index.html and the rotator is reachable through the API only, and getting it back needs a firmware upload." />
  {:else}
    <!-- Only NVS gets the restart button: it is the answer to that panel's
         warning, and nothing on the filesystem side needs one. -->
    <Explorer store={stores.nvs}
              hint="Not a filesystem but keys and values: namespaces as folders, keys as files. The extension is a reading of the content, not a stored name. An update leaves NVS untouched — which is why the WiFi credentials and the expert password live here."
              warning="Some settings are cached in RAM and written back on the next change. An edit made here survives only an immediate restart."
              onRestart={restartRotator} />
  {/if}
</section>

<style>
  /*
   * NOT `.switch`: app.css already gives that name to the on/off toggle, with
   * a fixed `width: 2.6rem; height: 1.5rem`. Svelte's scoping adds a class, it
   * does not stop the global rule matching too - so the row inherited a 24 px
   * height it never asked for, the labels overflowed it, and the underline
   * came out through the middle of the word. Any class name here has to be one
   * app.css does not already use.
   *
   * Underlined tabs, the same idiom as the tab row at the top of the page.
   */
  .store-tabs {
    display: flex;
    gap: 0.25rem;
    margin: 0 0 1rem;
    border-bottom: 1px solid var(--border);
  }

  .store-tabs button {
    appearance: none;
    background: none;
    border: none;
    border-bottom: 2px solid transparent;
    margin-bottom: -1px;
    padding: 0.5rem 0.9rem;
    font: inherit;
    font-size: 0.9rem;
    color: var(--muted);
    cursor: pointer;
  }

  .store-tabs button:hover {
    color: var(--text);
  }

  .store-tabs button.on {
    color: var(--accent);
    border-bottom-color: var(--accent);
    font-weight: 600;
  }

  .store-tabs button:focus-visible {
    outline: 2px solid var(--accent);
    outline-offset: -2px;
  }
</style>
