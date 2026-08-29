<script>
  /**
   * Expert
   * Setting, entering and clearing the password that unlocks Update, Debug
   * and Storage.
   *
   * Not a tab of its own: there is nothing here for someone who has not gone
   * looking, and a chip in the row would only invite guessing. It is reached
   * through `#expert` in the address, which App.svelte watches for.
   *
   * The three states it can be in - not enrolled, locked, unlocked - are the
   * rotator's, not this component's, and they arrive as a prop from a fresh
   * /expert on the way in: a second browser that unlocked the rotator a
   * moment ago is noticed here rather than argued with.
   */
  import { enrollExpert, unlockExpert, lockExpert, resetExpert } from '../lib/api.js';

  let { expert, onchange } = $props();

  const MIN_LENGTH = 6; // the firmware refuses anything shorter

  let password = $state('');
  let busy = $state(false);
  let error = $state('');

  async function submit() {
    if (busy || password.length < MIN_LENGTH) return;
    busy = true;
    error = '';
    try {
      const answer = expert.enrolled ? await unlockExpert(password) : await enrollExpert(password);
      password = '';
      onchange(answer);
    } catch (err) {
      // Cleared either way. A wrong one is not worth offering again
      // unchanged, and a right one has no business staying in a form field.
      password = '';
      error = err.message;
    } finally {
      busy = false;
    }
  }

  async function lock() {
    busy = true;
    try { onchange(await lockExpert()); }
    catch (err) { error = err.message; }
    finally { busy = false; }
  }

  async function clearPassword() {
    busy = true;
    error = '';
    try { onchange(await resetExpert()); }
    catch (err) { error = err.message; }
    finally { busy = false; }
  }

  /** The countdown on the reset window, as m:ss. */
  const graceText = $derived(
    `${Math.floor(expert.grace / 60)}:${String(expert.grace % 60).padStart(2, '0')}`
  );
</script>

<section class="card">
  <h2>Expert mode</h2>

  {#if expert.unlocked}
    <p class="hint success">Update, Debug and Storage are unlocked.</p>
    <div class="actions">
      <button type="button" class="secondary" disabled={busy} onclick={lock}>Lock again</button>
    </div>
  {:else}
    <p class="hint">
      {expert.enrolled
        ? 'Enter the expert password to unlock Update, Debug and Storage.'
        : 'Set an expert password to unlock Update, Debug and Storage.'}
    </p>

    <div class="field">
      <label for="expert-password">Password</label>
      <input
        id="expert-password"
        type="password"
        autocomplete={expert.enrolled ? 'current-password' : 'new-password'}
        bind:value={password}
        disabled={busy || expert.lockedOut}
        onkeydown={(e) => e.key === 'Enter' && submit()}
      />
    </div>

    <div class="actions">
      <button type="button" disabled={busy || expert.lockedOut || password.length < MIN_LENGTH} onclick={submit}>
        {expert.enrolled ? 'Unlock' : 'Set password'}
      </button>
    </div>

    {#if !expert.enrolled}<p class="hint">At least {MIN_LENGTH} characters.</p>{/if}
    {#if expert.lockedOut}<p class="banner">Too many attempts. Try again in five minutes.</p>{/if}
  {/if}

  {#if error}<p class="banner">{error}</p>{/if}
</section>

{#if expert.grace > 0 && expert.enrolled}
  <!--
    Only while the window is open, and the window only opens on a power-on
    reset - so this card is not something a visitor can conjure up over the
    network. It is the way back from a forgotten password: pull the power,
    reconnect within five minutes, and the password can be cleared here
    without knowing it.
  -->
  <section class="card">
    <h2>Forgot the password?</h2>
    <p class="hint">
      This rotator just power-cycled, so the password can still be cleared
      without it — {graceText} left before the window closes.
    </p>
    <div class="actions">
      <button type="button" class="danger" disabled={busy} onclick={clearPassword}>
        Clear expert password
      </button>
    </div>
  </section>
{/if}

<style>
  .actions {
    display: flex;
    gap: 0.5rem;
    margin-top: 0.75rem;
  }

  .success {
    color: var(--accent);
  }

  .danger {
    font: inherit;
    padding: 0.5rem 1.1rem;
    border: 1px solid var(--danger);
    border-radius: 7px;
    background: var(--danger-bg);
    color: var(--danger);
    cursor: pointer;
  }

  .danger:disabled {
    opacity: 0.5;
    cursor: default;
  }
</style>
