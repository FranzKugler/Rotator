class AngleGauge {
  constructor(svgEl, { step = 15, size = 300, direction = 'cw' } = {}) {
    this.svg = svgEl;
    this.size = size;
    this.radius = size / 2 - 20;
    this.center = size / 2;
    this.step = step;
    this.direction = direction;
    this.offset = -90; // 0° oben
    this._init();
  }

  _init() {
    this.svg.innerHTML = '';
    this.svg.setAttribute('viewBox', `0 0 ${this.size} ${this.size}`);
    const ns = 'http://www.w3.org/2000/svg';

    // Grundkreis
    const circle = document.createElementNS(ns, 'circle');
    circle.setAttribute('cx', this.center);
    circle.setAttribute('cy', this.center);
    circle.setAttribute('r', this.radius);
    circle.setAttribute('fill', 'none');
    circle.setAttribute('stroke', '#666');
    this.svg.appendChild(circle);

    // Tick-Marks und Beschriftung
    for (let a = 0; a < 360; a += this.step) {
      // Winkel plus Offset und ggf. invertieren
      const raw = this.direction === 'cw' ? a : 360 - a;
      const θ = (Math.PI / 180) * (raw + this.offset);

      const x1 = this.center + Math.cos(θ) * this.radius;
      const y1 = this.center + Math.sin(θ) * this.radius;
      const tickLen = (a % 90 === 0 ? 15 : 10);
      const x2 = this.center + Math.cos(θ) * (this.radius - tickLen);
      const y2 = this.center + Math.sin(θ) * (this.radius - tickLen);

      const line = document.createElementNS(ns, 'line');
      line.setAttribute('x1', x1);
      line.setAttribute('y1', y1);
      line.setAttribute('x2', x2);
      line.setAttribute('y2', y2);
      line.setAttribute('stroke', '#333');
      this.svg.appendChild(line);

      if (a % 45 === 0) {
        const labelDist = this.radius - 30;
        const tx = this.center + Math.cos(θ) * labelDist;
        const ty = this.center + Math.sin(θ) * labelDist;
        const text = document.createElementNS(ns, 'text');
        text.setAttribute('x', tx);
        text.setAttribute('y', ty + 5);
        text.setAttribute('text-anchor', 'middle');
        text.setAttribute('font-size', '14');
        text.textContent = a;
        this.svg.appendChild(text);
      }
    }
    // Roter, gefüllter Kreis in der Mitte (Durchmesser 20)
    const centerCircle = document.createElementNS(ns, 'circle');
    centerCircle.setAttribute('cx', this.center);
    centerCircle.setAttribute('cy', this.center);
    centerCircle.setAttribute('r', 10);
    centerCircle.setAttribute('fill', 'red');
    this.svg.appendChild(centerCircle);

    // Pointer hinzufügen
    this.pointer = document.createElementNS(ns, 'line');
    this.pointer.setAttribute('x1', this.center);
    this.pointer.setAttribute('y1', this.center);
    this.pointer.setAttribute('stroke', 'red');
    this.pointer.setAttribute('stroke-width', '3');
    this.svg.appendChild(this.pointer);
  }

  setDirection(dir) {

    if (this.direction !== dir) {
      this.direction = dir;
      this._init();
    }
  }

  update(angle) {
    const raw = this.direction === 'cw' ? angle : 360 - angle;
    const θ = (Math.PI / 180) * (raw + this.offset);
    const x1 = this.center - Math.cos(θ) * 20;
    const y1 = this.center - Math.sin(θ) * 20;
    const x2 = this.center + Math.cos(θ) * (this.radius - 20);
    const y2 = this.center + Math.sin(θ) * (this.radius - 20);
    this.pointer.setAttribute('x1', x1);
    this.pointer.setAttribute('y1', y1);
    this.pointer.setAttribute('x2', x2);
    this.pointer.setAttribute('y2', y2);
  }
}

// Initialisierung per SSE …
document.addEventListener('DOMContentLoaded', () => {
  const svg = document.getElementById('angleGauge');
  const gauge = new AngleGauge(svg, { step: 15, size: 300, direction: 'cw' });

  const angleLabel = document.getElementById('angleVal');
  const mechLabel = document.getElementById('mechVal');

  // WebSocket connection
  const socket = new WebSocket(`ws://${location.host}/api/info/events`);
  socket.addEventListener('message', ({ data }) => {
    const { angle, mechAngle, direction } = JSON.parse(data);
    gauge.setDirection(direction);
    gauge.update(angle);
    angleLabel.textContent = angle;
    mechLabel.textContent = mechAngle;
  });
  socket.addEventListener('error', e => console.error('WebSocket Error:', e));
});
