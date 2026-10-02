import init, { ResonantCircuitSim } from './pkg/lccl_ss_wasm.js';

async function run() {
  await init();

  const sim = new ResonantCircuitSim();
  const canvas = document.getElementById('oscilloscope');
  const ctx = canvas.getContext('2d');
  const toggleBtn = document.getElementById('toggleBtn');

  // Draw zero reference baseline
  function drawGrid() {
    ctx.beginPath();
    ctx.strokeStyle = '#333333';
    ctx.setLineDash([4, 4]);
    ctx.moveTo(0, canvas.height / 2);
    ctx.lineTo(canvas.width, canvas.height / 2);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Helper to render a waveform relative to center zero
  function drawWaveform(data, color, scale = 10) {
    if (!data || data.length === 0) return;

    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;

    const centerY = canvas.height / 2;
    const stepX = canvas.width / (data.length - 1 || 1);

    for (let i = 0; i < data.length; i++) {
      const x = i * stepX;
      const y = centerY - data[i] * scale;

      if (i === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();
  }

  function renderLoop() {
    // Run 500 sub-steps in Rust WASM per animation frame (~16ms)
    sim.update(0.0001, 500);

    const ipHistory = sim.get_history_ip();
    const isHistory = sim.get_history_is();

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw reference zero-axis
    drawGrid();

    // Render waveforms
    drawWaveform(ipHistory, '#00ffff'); // Primary Current (ip) - Cyan
    drawWaveform(isHistory, '#ff00ff'); // Secondary Current (is) - Magenta

    requestAnimationFrame(renderLoop);
  }

  // Toggle topology mode on UI button click
  toggleBtn.addEventListener('click', () => {
    const isSS = toggleBtn.classList.toggle('active');
    sim.set_topology(isSS);
  });

  renderLoop();
}

run();