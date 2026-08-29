<script>
  /**
   * Explorer
   * The tree, the context menu and the editor, for whichever store it is given.
   *
   * One component draws both LittleFS and NVS - see lib/explorers.js for the
   * interface and for why the two are shown the same way. Everything that
   * differs between them arrives as a capability on the adapter rather than as
   * a branch in here: `canUpload` and `canMkdir` decide whether those buttons
   * exist at all, because a greyed-out button for something NVS cannot do in
   * principle would be an invitation to wonder why.
   *
   * State is kept flat - a path-keyed object of listings plus a set of open
   * paths - rather than as a recursive component, so a refresh has one place to
   * touch and a delete three levels down does not have to find its own node.
   */
  import { onMount, onDestroy } from 'svelte';

  let { store, hint = '', warning = '', onRestart = null } = $props();

  let dirs = $state({});
  let open = $state({ '/': true });
  let volume = $state(null);
  let error = $state(null);
  let note = $state(null);
  let busy = $state(null);

  /** Where an upload or a new folder would land: the last folder chosen. */
  let target = $state('/');
  let progress = $state(null);
  let creating = $state(null);
  let picker = $state(null);

  /** The open context menu: which row, and where to draw it. */
  let menu = $state(null);
  let editing = $state(null);

  const join = (dir, name) => (dir === '/' ? `/${name}` : `${dir}/${name}`);

  const parentOf = (path) => {
    const cut = path.lastIndexOf('/');
    return cut <= 0 ? '/' : path.substring(0, cut);
  };

  async function load(path) {
    try {
      const answer = await store.list(path);
      volume = { used: answer.used, total: answer.total };
      // Folders first, then by name - the order either store hands entries out
      // in is the order they were written, which is no order at all to read.
      const entries = [...answer.entries].sort((a, b) =>
        a.dir === b.dir ? a.name.localeCompare(b.name) : a.dir ? -1 : 1
      );
      dirs = { ...dirs, [path]: { entries, truncated: !!answer.truncated } };
      error = null;
    } catch (err) {
      error = err.message;
    }
  }

  /* Switching stores destroys this component and builds a new one - the two
     live in different branches of Storage.svelte - so there is nothing to
     reset here and no effect watching the prop. */
  onMount(() => load('/'));

  async function toggle(path) {
    target = path;
    if (open[path]) {
      open = { ...open, [path]: false };
      return;
    }
    open = { ...open, [path]: true };
    if (!dirs[path]) await load(path);
  }

  const rows = $derived.by(() => {
    const out = [];
    const walk = (path, depth) => {
      const here = dirs[path];
      if (!here) return;
      for (const entry of here.entries) {
        const full = join(path, entry.name);
        out.push({ ...entry, path: full, depth });
        if (entry.dir && open[full]) walk(full, depth + 1);
      }
      if (here.truncated) out.push({ truncated: true, path: `${path}#more`, depth });
    };
    walk('/', 0);
    return out;
  });

  // ------ the context menu ------

  /*
   * Right-click on a pointer, long press on a touch screen. There is no third
   * gesture that means "what can I do with this" on a phone: a swipe collides
   * with the page scroller, and an extra button per row is exactly the clutter
   * this replaced. Keyboards get Enter and the menu key, so the rows are
   * focusable and the tree is usable without a mouse at all.
   */
  const LONG_PRESS_MS = 500;
  let pressTimer = null;
  /* A long press ends in a click, which the window handler below would read as
     "clicked outside" and close the menu again the instant it appeared. The
     same goes for the keyboard, where the keydown is followed by a click on
     some browsers. So the window ignores whatever arrives immediately after. */
  let openedAt = 0;

  function openMenu(row, x, y) {
    if (row.truncated) return;
    openedAt = Date.now();
    // Clamped so a menu opened at the right edge does not leave the viewport.
    menu = {
      row,
      x: Math.min(x, window.innerWidth - 190),
      y: Math.min(y, window.innerHeight - 190)
    };
  }

  function closeOnOutside() {
    if (Date.now() - openedAt < 300) return;
    menu = null;
  }

  function onContext(event, row) {
    event.preventDefault();
    openMenu(row, event.clientX, event.clientY);
  }

  function onPointerDown(event, row) {
    if (event.pointerType !== 'touch') return;
    clearTimeout(pressTimer);
    pressTimer = setTimeout(() => {
      // A long press that becomes a menu must not also become a tap, and on
      // touch the browser would otherwise fire both.
      event.target.releasePointerCapture?.(event.pointerId);
      openMenu(row, event.clientX, event.clientY);
    }, LONG_PRESS_MS);
  }

  const cancelPress = () => clearTimeout(pressTimer);

  function onKey(event, row) {
    // Only when the row itself has focus. A folder row holds a button, and
    // Enter on that means "open the folder" - letting it bubble up to here
    // would open the folder and the menu at once.
    if (event.target !== event.currentTarget) return;
    if (event.key === 'ContextMenu' || event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      const box = event.currentTarget.getBoundingClientRect();
      openMenu(row, box.left + 24, box.bottom);
    }
  }

  function onWindowKey(event) {
    if (event.key === 'Escape') {
      if (menu) menu = null;
      else if (creating !== null) creating = null;
    }
  }

  onDestroy(() => clearTimeout(pressTimer));

  // ------ the actions the menu offers ------

  async function remove(row) {
    menu = null;
    if (!confirm(`Delete "${row.name}" for good?`)) return;

    busy = row.path;
    try {
      await store.remove(row.path);
      const { [row.path]: _gone, ...rest } = dirs;
      dirs = rest;
      await load(parentOf(row.path));
      if (editing?.path === row.path) editing = null;
      note = null;
      error = null;
    } catch (err) {
      error = err.message;
    } finally {
      busy = null;
    }
  }

  async function edit(row) {
    menu = null;
    busy = row.path;
    try {
      const text = await store.readText(row.path);
      editing = { path: row.path, name: row.name, ...prettify(text) };
      note = null;
      error = null;
    } catch (err) {
      error = err.message;
    } finally {
      busy = null;
    }
  }

  async function save() {
    editing.saving = true;
    try {
      await store.save(editing.path, outgoing(editing));
      await load(parentOf(editing.path));
      note = 'Saved.';
      editing = null;
      error = null;
    } catch (err) {
      error = err.message;
    } finally {
      if (editing) editing.saving = false;
    }
  }

  /*
   * JSON arrives from both stores as one line - that is how the firmware
   * writes its records and how a build tool writes version.json - and one
   * line is not something anybody can read, let alone correct. So it is laid
   * out on the way in.
   *
   * What matters is the way back out. A record shown pretty and saved pretty
   * is three times its size on a partition that has a job to do, so a value
   * that arrived compact goes back compact unless the box says otherwise. The
   * choice is visible rather than magic: hiding it would mean the editor
   * silently deciding what to write.
   */
  function prettify(text) {
    try {
      const parsed = JSON.parse(text);
      const pretty = JSON.stringify(parsed, null, 2);
      if (pretty === text) return { text, json: true, compact: false };
      return { text: pretty, json: true, compact: !text.includes('\n') };
    } catch {
      return { text, json: false, compact: false };
    }
  }

  /** What actually goes to the rotator, given the compact switch. */
  function outgoing(state) {
    if (!state.json || !state.compact) return state.text;
    try {
      return JSON.stringify(JSON.parse(state.text));
    } catch {
      // No longer valid JSON, so it cannot be minified - and refusing to save
      // half-typed text is worse than saving exactly what is on screen.
      return state.text;
    }
  }

  /** Whether the box still holds JSON, for the badge beside the switch. */
  const editorValid = $derived.by(() => {
    if (!editing?.json) return true;
    try {
      JSON.parse(editing.text);
      return true;
    } catch {
      return false;
    }
  });

  let restarting = $state(false);

  async function restart() {
    if (!confirm('Restart the rotator now? It will be unreachable for a few seconds.')) return;
    restarting = true;
    try {
      await onRestart();
      note = 'Restart asked for — reload the page in a few seconds.';
      error = null;
    } catch (err) {
      error = err.message;
      restarting = false;
    }
    // Left true on success: the rotator is going away for a few seconds, and
    // the button must not invite a second press while it does.
  }

  // ------ the two writes only LittleFS has ------

  async function upload(event) {
    const file = event.target.files?.[0];
    // Cleared straight away, or picking the same file twice fires nothing.
    event.target.value = '';
    if (!file) return;

    progress = 0;
    try {
      await store.upload(join(target, file.name), file, (done) => {
        progress = Math.round(done * 100);
      });
      await load(target);
      open = { ...open, [target]: true };
      error = null;
    } catch (err) {
      error = err.message;
    } finally {
      progress = null;
    }
  }

  async function createFolder() {
    const name = creating.trim();
    if (!name) {
      creating = null;
      return;
    }
    try {
      await store.mkdir(join(target, name));
      await load(target);
      open = { ...open, [target]: true };
      creating = null;
      error = null;
    } catch (err) {
      error = err.message;
    }
  }

  /*
   * Three different things get a number beside them, and conflating them is
   * how "curve.json  80 entries" happened: a value 80 bytes long, labelled
   * with the unit the volume is measured in.
   *
   *   a value      always bytes, in both stores
   *   the volume   bytes on a filesystem, 32-byte entries in NVS
   *   a folder     nothing on a filesystem, its key count in NVS
   */
  const bytes = (n) =>
    n < 1024 ? `${n} B`
    : n < 1048576 ? `${(n / 1024).toFixed(1)} kB`
    : `${(n / 1048576).toFixed(2)} MB`;

  const usage = $derived(
    !volume
      ? ''
      : store.unit === 'entries'
        ? `${volume.used} of ${volume.total} entries used`
        : `${bytes(volume.used)} of ${bytes(volume.total)} used`
  );

  /** What sits at the right of a row. A folder only has something to say in NVS. */
  const sizeOf = (row) =>
    !row.dir ? bytes(row.size) : store.unit === 'entries' ? `${row.size} keys` : '';
</script>

<svelte:window onkeydown={onWindowKey} onclick={closeOnOutside} onscroll={closeOnOutside} />

<p class="hint">{hint}</p>
{#if warning}
  <p class="hint warn">
    {warning}
    <!-- Beside the sentence rather than in a row of its own: it is the answer
         to what the sentence just said, and a restart button anywhere else on
         this page would be an invitation rather than a remedy. -->
    {#if onRestart}
      <button type="button" class="restart" onclick={restart} disabled={restarting}>
        {restarting ? 'Restarting …' : 'Restart now'}
      </button>
    {/if}
  </p>
{/if}

{#if error}<p class="banner">{error}</p>{/if}
{#if note}<p class="hint ok">{note}</p>{/if}

{#if volume}
  <div class="field">
    <span class="key">{usage}</span>
    <span class="bar" aria-hidden="true">
      <span style="width: {Math.min(100, (volume.used / volume.total) * 100)}%"></span>
    </span>
  </div>
{/if}

{#if store.canUpload || store.canMkdir}
  <div class="actions">
    {#if store.canUpload}
      <button type="button" onclick={() => picker.click()} disabled={progress !== null}>
        {progress === null ? 'Upload' : `Transferring … ${progress} %`}
      </button>
      <input type="file" bind:this={picker} onchange={upload} hidden />
    {/if}
    {#if store.canMkdir}
      <button type="button" onclick={() => (creating = '')} disabled={creating !== null}>
        New folder
      </button>
    {/if}
    <!-- Which folder the buttons act on. Clicking a folder moves it. -->
    <code class="target">{target}</code>
  </div>

  {#if creating !== null}
    <div class="actions">
      <!-- svelte-ignore a11y_autofocus -->
      <input type="text" bind:value={creating} placeholder="Folder name" autofocus
             onkeydown={(e) => e.key === 'Enter' && createFolder()} />
      <button type="button" onclick={createFolder}>Save</button>
      <button type="button" onclick={() => (creating = null)}>Cancel</button>
    </div>
  {/if}
{/if}

<div class="tree" role="tree">
  <div class="row" class:here={target === '/'} role="treeitem" aria-selected={target === '/'}>
    <button type="button" class="name" onclick={() => (target = '/')}>
      <span class="twist">▾</span><span class="folder">Root</span>
    </button>
  </div>

  {#each rows as row (row.path)}
    {#if row.truncated}
      <div class="row muted" style="--depth: {row.depth + 1}">Only the first entries — the folder holds more.</div>
    {:else}
      <div class="row" class:here={target === row.path} class:busy={busy === row.path}
           style="--depth: {row.depth + 1}"
           role="treeitem" tabindex="0"
           aria-selected={target === row.path}
           aria-expanded={row.dir ? !!open[row.path] : undefined}
           oncontextmenu={(e) => onContext(e, row)}
           onpointerdown={(e) => onPointerDown(e, row)}
           onpointerup={cancelPress} onpointermove={cancelPress} onpointercancel={cancelPress}
           onkeydown={(e) => onKey(e, row)}>
        {#if row.dir}
          <button type="button" class="name" onclick={() => toggle(row.path)}>
            <span class="twist">{open[row.path] ? '▾' : '▸'}</span>
            <span class="folder">{row.name}</span>
          </button>
        {:else}
          <span class="name file"><span class="twist"></span>{row.name}</span>
        {/if}
        <span class="size">{sizeOf(row)}</span>
      </div>
    {/if}
  {/each}

  {#if dirs['/'] && dirs['/'].entries.length === 0}
    <p class="hint">This folder is empty.</p>
  {/if}
</div>

<p class="hint gesture">Right-click, or press and hold, to open the menu.</p>

{#if menu}
  <!-- No click handler of its own: choosing an item closes the menu on the way
       past through the window handler, which is what should happen anyway. -->
  <div class="menu" style="left: {menu.x}px; top: {menu.y}px" role="menu">
    <div class="menu-title">{menu.row.name}</div>
    {#if !menu.row.dir}
      <a role="menuitem" href={store.urlOf(menu.row.path)} download={menu.row.name}
         class:disabled={menu.row.protected} onclick={() => (menu = null)}>Download</a>
      {#if menu.row.edit}
        <button type="button" role="menuitem" onclick={() => edit(menu.row)}>Edit</button>
      {:else}
        <button type="button" role="menuitem" disabled>
          {menu.row.protected ? 'Not readable (password)' : 'Not text — download only.'}
        </button>
      {/if}
    {/if}
    {#if menu.row.dir && store.canMkdir}
      <button type="button" role="menuitem"
              onclick={() => { target = menu.row.path; menu = null; creating = ''; }}>
        New folder inside
      </button>
    {/if}
    {#if !(menu.row.dir && store.key === 'nvs')}
      <button type="button" role="menuitem" class="danger"
              onclick={() => remove(menu.row)}>Delete</button>
    {/if}
  </div>
{/if}

{#if editing}
  <div class="editor">
    <div class="editor-head">
      <code class="target">{editing.path}</code>
      {#if editing.json}
        <label class="compact">
          <input type="checkbox" bind:checked={editing.compact} />
          save compact
        </label>
        {#if !editorValid}<span class="invalid">not valid JSON</span>{/if}
      {/if}
    </div>
    <textarea bind:value={editing.text} spellcheck="false" rows="18"></textarea>
    <div class="actions">
      <button type="button" onclick={save} disabled={editing.saving}>Save</button>
      <button type="button" onclick={() => (editing = null)}>Cancel</button>
    </div>
  </div>
{/if}

<style>
  .bar {
    flex: 1;
    height: 8px;
    margin-left: 0.75rem;
    border-radius: 4px;
    background: var(--border);
    overflow: hidden;
  }

  .bar > span {
    display: block;
    height: 100%;
    background: var(--accent);
  }

  .actions {
    display: flex;
    gap: 0.5rem;
    align-items: center;
    flex-wrap: wrap;
    margin: 0.75rem 0;
  }

  .target {
    font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    font-size: 0.8rem;
    color: var(--muted);
  }

  .tree {
    border: 1px solid var(--border);
    border-radius: var(--radius);
    background: var(--bg);
    padding: 0.35rem 0.5rem;
    font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    font-size: 0.82rem;
    /* A deep tree or a long name scrolls here rather than widening the page. */
    overflow-x: auto;
  }

  .row {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    padding: 0.15rem 0;
    padding-left: calc(var(--depth, 0) * 1.1rem);
    border-radius: 4px;
    /* A long press must open the menu, not select the row's text. */
    -webkit-user-select: none;
    user-select: none;
    /* And it must not raise the browser's own callout on iOS. */
    -webkit-touch-callout: none;
  }

  .row:focus-visible {
    outline: 2px solid var(--accent);
    outline-offset: -2px;
  }

  .row.here {
    background: color-mix(in srgb, var(--accent) 12%, transparent);
  }

  .row.busy {
    opacity: 0.5;
  }

  .row.muted {
    color: var(--muted);
    font-style: italic;
  }

  .name {
    display: flex;
    align-items: center;
    gap: 0.35rem;
    flex: 1;
    min-width: 0;
    /* The folder row is a button, so it needs stripping back to a plain row. */
    background: none;
    border: none;
    padding: 0;
    margin: 0;
    font: inherit;
    color: inherit;
    text-align: left;
    cursor: pointer;
    white-space: nowrap;
  }

  .name.file {
    cursor: default;
  }

  .twist {
    display: inline-block;
    width: 1em;
    flex: none;
    color: var(--muted);
  }

  .folder {
    font-weight: 600;
  }

  .size {
    color: var(--muted);
    font-variant-numeric: tabular-nums;
    flex: none;
  }

  .gesture {
    font-size: 0.78rem;
  }

  .menu {
    position: fixed;
    z-index: 40;
    min-width: 11rem;
    display: flex;
    flex-direction: column;
    padding: 0.25rem;
    border: 1px solid var(--border);
    border-radius: var(--radius);
    background: var(--surface);
    box-shadow: 0 8px 24px rgb(0 0 0 / 0.18);
  }

  .menu-title {
    padding: 0.25rem 0.5rem 0.35rem;
    color: var(--muted);
    font-size: 0.72rem;
    font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    border-bottom: 1px solid var(--border);
    margin-bottom: 0.2rem;
    /* A long key must not stretch the menu across the screen. */
    max-width: 14rem;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .menu button,
  .menu a {
    display: block;
    width: 100%;
    box-sizing: border-box;
    text-align: left;
    background: none;
    border: none;
    border-radius: 4px;
    padding: 0.35rem 0.5rem;
    font: inherit;
    font-size: 0.85rem;
    color: inherit;
    text-decoration: none;
    cursor: pointer;
  }

  .menu button:hover:not(:disabled),
  .menu a:hover:not(.disabled) {
    background: color-mix(in srgb, var(--accent) 14%, transparent);
  }

  .menu button:disabled,
  .menu a.disabled {
    color: var(--muted);
    cursor: default;
    pointer-events: none;
  }

  .menu .danger {
    color: var(--danger);
  }

  .editor {
    margin-top: 1rem;
  }

  .editor-head {
    display: flex;
    align-items: center;
    gap: 0.75rem;
    flex-wrap: wrap;
    margin-bottom: 0.4rem;
  }

  .compact {
    display: flex;
    align-items: center;
    gap: 0.3rem;
    font-size: 0.8rem;
    color: var(--muted);
  }

  .invalid {
    font-size: 0.8rem;
    color: var(--danger);
  }

  .editor textarea {
    width: 100%;
    box-sizing: border-box;
    font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    font-size: 0.8rem;
    line-height: 1.45;
    /* No wrapping: a record is read by its columns once it is laid out. */
    white-space: pre;
    overflow-wrap: normal;
    overflow-x: auto;
  }

  .restart {
    /* Inline with the warning it answers, so it reads as part of the sentence
       rather than as a control that happens to sit nearby. */
    margin-left: 0.5rem;
    padding: 0.15rem 0.6rem;
    font-size: 0.78rem;
    vertical-align: baseline;
  }

  .hint.ok {
    color: var(--accent);
  }

  .hint.warn {
    /* Not a token: amber is wanted in both themes and is legible on both. */
    color: #b26a00;
  }

  @media (prefers-color-scheme: dark) {
    .hint.warn {
      color: #e0a350;
    }
  }
</style>
