// Shared by tablet, screen and operator pages: websocket, i18n, board drawing.

const T = {
  fi: {
    press_play: "Haasta robotti!",
    play: "Pelaa",
    your_turn: "Sinun vuorosi – olet X",
    thinking: "Robotti miettii…",
    robot_moving: "Robotti tekee siirtonsa",
    robot_wins: "Robotti voitti!",
    draw: "Tasapeli!",
    you_win: "Voitit robotin!",
    pick_colour: "Voitit! Minkä värisen suklaan haluat?",
    draw_pick_colour: "Tasapeli! Saat silti suklaan – minkä värin valitset?",
    giving: "Robotti hakee suklaasi…",
    enjoy: "Ole hyvä! Ota suklaa kupista.",
    no_chocolate: "Voitit! Suklaat loppuivat hetkeksi.",
    stopped: "Robotti on pysäytetty",
    calib: "Kalibrointi käynnissä",
    win: "voitto", lose: "häviö", draw_s: "tasapeli",
    futures_caption: "mahdollista peliä robotti pelasi päässään loppuun ennen siirtoa",
    futures_idle: "Robotti laskee jokaisen mahdollisen pelin loppuun asti.",
    students: "Oppilaat", robot: "Robotti", draws: "Tasapelit",
    red: "punainen", blue: "sininen", green: "vihreä", yellow: "keltainen",
    legend: "Robotin arvio jokaisesta vapaasta ruudusta",
  },
  en: {
    press_play: "Challenge the robot!",
    play: "Play",
    your_turn: "Your turn – you are X",
    thinking: "The robot is thinking…",
    robot_moving: "The robot makes its move",
    robot_wins: "The robot wins!",
    draw: "It's a draw!",
    you_win: "You beat the robot!",
    pick_colour: "You won! Which colour chocolate do you want?",
    draw_pick_colour: "A draw! You still get a chocolate – which colour?",
    giving: "The robot is fetching your chocolate…",
    enjoy: "Here you go! Take it from the cup.",
    no_chocolate: "You won! We're out of chocolate for a moment.",
    stopped: "The robot is stopped",
    calib: "Calibrating",
    win: "win", lose: "lose", draw_s: "draw",
    futures_caption: "possible games the robot played out in its head before moving",
    futures_idle: "The robot plays every possible game to the end in its head.",
    students: "Students", robot: "Robot", draws: "Draws",
    red: "red", blue: "blue", green: "green", yellow: "yellow",
    legend: "Robot's verdict for each free square",
  },
};

function t(state, key, arg) {
  const d = T[state?.lang] || T.fi;
  const v = d[key] ?? T.en[key] ?? key;
  return typeof v === "function" ? v(arg) : v;
}

// ---- websocket with auto-reconnect ---------------------------------------
function connect(role, onState) {
  let ws;
  const status = document.getElementById("conn");
  function open() {
    ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws?role=${role}`);
    ws.onopen = () => status && (status.hidden = true);
    ws.onmessage = e => {
      const m = JSON.parse(e.data);
      if (m.type === "state") onState(m);
    };
    ws.onclose = () => {
      if (status) status.hidden = false;
      setTimeout(open, 1000);
    };
  }
  open();
  return msg => ws && ws.readyState === 1 && ws.send(JSON.stringify(msg));
}

// ---- board -----------------------------------------------------------------
const SVGNS = "http://www.w3.org/2000/svg";

function markSVG(v) {
  const s = document.createElementNS(SVGNS, "svg");
  s.setAttribute("viewBox", "0 0 100 100");
  s.classList.add("mark", v === "X" ? "mark-x" : "mark-o");
  s.innerHTML = v === "X"
    ? '<path d="M24 24 L76 76 M76 24 L24 76"/>'
    : '<circle cx="50" cy="50" r="28"/>';
  return s;
}

// Builds a 3x3 board inside `el`; returns render(state, opts).
function makeBoard(el, onCell) {
  el.classList.add("board");
  const cells = [];
  for (let i = 1; i <= 9; i++) {
    const c = document.createElement("div");
    c.className = "cell";
    c.dataset.cell = i;
    if (onCell) c.addEventListener("pointerdown", e => { e.preventDefault(); onCell(i); });
    el.appendChild(c);
    cells.push(c);
  }
  const line = document.createElementNS(SVGNS, "svg");
  line.setAttribute("viewBox", "0 0 300 300");
  line.classList.add("winline");
  el.appendChild(line);
  let lastKey = "";

  return function render(st, opts = {}) {
    st.board.forEach((v, i) => {
      const c = cells[i];
      const have = c.querySelector(".mark");
      if (v && !have) c.appendChild(markSVG(v));
      if (!v && have) have.remove();
      c.classList.toggle("target", st.robot_cell === i + 1 && (st.phase === "thinking" || st.phase === "robot"));
      const verdict = opts.scores && !v ? st.scores[String(i + 1)] : null;
      let tag = c.querySelector(".verdict");
      if (verdict) {
        if (!tag) { tag = document.createElement("span"); c.appendChild(tag); }
        tag.className = "verdict v-" + verdict;
        tag.textContent = t(st, verdict === "draw" ? "draw_s" : verdict);
      } else if (tag) tag.remove();
    });
    const key = JSON.stringify(st.line);
    if (key !== lastKey) {
      lastKey = key;
      line.innerHTML = "";
      if (st.line) {
        const p = n => [((n - 1) % 3) * 100 + 50, Math.floor((n - 1) / 3) * 100 + 50];
        const [a, b] = [p(st.line[0]), p(st.line[2])];
        const winner = st.board[st.line[0] - 1];
        line.innerHTML = `<line class="${winner === "X" ? "lx" : "lo"}" x1="${a[0]}" y1="${a[1]}" x2="${b[0]}" y2="${b[1]}"/>`;
      }
    }
  };
}

function messageKey(st) {
  if (st.phase === "result" && st.result === "X") return "you_win";
  return st.message;
}
