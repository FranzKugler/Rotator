<script>
  let { angle = 0, direction = 'cw' } = $props();
  const ticks = Array.from({ length: 24 }, (_, index) => index * 15);
  const polar = (degrees, radius) => {
    const raw = direction === 'cw' ? degrees : 360 - degrees;
    const radians = (raw - 90) * Math.PI / 180;
    return [150 + Math.cos(radians) * radius, 150 + Math.sin(radians) * radius];
  };
  const pointer = $derived.by(() => {
    const raw = direction === 'cw' ? angle : 360 - angle;
    const radians = (raw - 90) * Math.PI / 180;
    return {
      x1: 150 - Math.cos(radians) * 18,
      y1: 150 - Math.sin(radians) * 18,
      x2: 150 + Math.cos(radians) * 105,
      y2: 150 + Math.sin(radians) * 105
    };
  });
</script>

<svg viewBox="0 0 300 300" role="img" aria-label={`Rotator angle ${Number(angle).toFixed(2)} degrees`}>
  <circle class="face" cx="150" cy="150" r="130" />
  {#each ticks as tick}
    {@const outer = polar(tick, 130)}
    {@const inner = polar(tick, tick % 90 === 0 ? 112 : 119)}
    <line class:major={tick % 90 === 0} x1={outer[0]} y1={outer[1]} x2={inner[0]} y2={inner[1]} />
    {#if tick % 45 === 0}
      {@const label = polar(tick, 98)}
      <text x={label[0]} y={label[1] + 5}>{tick}</text>
    {/if}
  {/each}
  <circle class="sweep" cx="150" cy="150" r="116" />
  <line class="pointer" x1={pointer.x1} y1={pointer.y1} x2={pointer.x2} y2={pointer.y2} />
  <circle class="hub" cx="150" cy="150" r="9" />
</svg>

<style>
  svg { width: min(100%, 22rem); filter: drop-shadow(0 18px 32px rgb(0 0 0 / .28)); }
  .face { fill: #251a3c; stroke: #79628c; stroke-width: 2; }
  line { stroke: #79628c; stroke-width: 1.5; }
  line.major { stroke: #fff; stroke-width: 2.5; }
  text { fill: #c7bfd2; text-anchor: middle; font: 600 13px Rubik, system-ui; }
  .sweep { fill: none; stroke: #e34b4b; stroke-width: 1; stroke-dasharray: 3 8; opacity: .58; }
  .pointer { stroke: #e34b4b; stroke-width: 4; stroke-linecap: round; }
  .hub { fill: #e34b4b; stroke: #ffc1c1; stroke-width: 2; }
</style>
