(() => {
  const navWrap = document.querySelector(".site-nav");
  const toc = document.getElementById("TOC");
  const toggle = document.querySelector(".toc-toggle");
  const fixEscapedAssetPaths = () => {
    const images = Array.from(document.querySelectorAll('img[src*="\\\\_"]'));
    for (const image of images) {
      image.setAttribute("src", image.getAttribute("src").replace(/\\_/g, "_"));
    }
  };

  const initChapterOneWorkflow = () => {
    const maps = Array.from(document.querySelectorAll("[data-workflow-map]"));
    for (const map of maps) {
      const buttons = Array.from(map.querySelectorAll(".workflow-hotspot"));
      const title = map.querySelector("[data-workflow-output-title]");
      const text = map.querySelector("[data-workflow-output-text]");
      const chapters = map.querySelector("[data-workflow-output-chapters]");
      const select = (button) => {
        for (const item of buttons) item.classList.remove("is-active");
        button.classList.add("is-active");
        if (title) title.textContent = button.dataset.workflowTitle || "";
        if (text) text.textContent = button.dataset.workflowText || "";
        if (chapters) chapters.textContent = button.dataset.workflowChapters || "";
      };
      for (const button of buttons) {
        button.addEventListener("click", () => select(button));
      }
    }
  };

  const initChapterOneDisturbance = () => {
    const panels = Array.from(document.querySelectorAll(".ch1-feedback-figure"));
    for (const panel of panels) {
      const time = panel.querySelector("[data-ch1-disturbance-time]");
      const size = panel.querySelector("[data-ch1-disturbance-size]");
      const openLoop = panel.querySelector("[data-ch1-open-loop]");
      const feedback = panel.querySelector("[data-ch1-feedback]");
      const line = panel.querySelector("[data-ch1-disturbance-line]");
      const arrow = panel.querySelector("[data-ch1-disturbance-arrow]");
      const label = panel.querySelector("[data-ch1-disturbance-label]");
      const openEnd = panel.querySelector("[data-ch1-open-end]");
      const feedbackEnd = panel.querySelector("[data-ch1-feedback-end]");
      const openLabel = panel.querySelector("[data-ch1-open-label]");
      const feedbackLabel = panel.querySelector("[data-ch1-feedback-label]");

      const update = () => {
        if (!time || !size || !openLoop || !feedback) return;
        const t0 = Number.parseFloat(time.value || "550") / 1000;
        const mag = Number.parseFloat(size.value || "0.75");
        const plantA = 7;
        const feedbackKp = 18;
        const feedbackKi = 150;
        const yScale = 430;
        const graphLeft = 84;
        const graphRight = 838;
        const tEnd = 1;
        const openBase = 120;
        const feedbackBase = 224;
        const x = graphLeft + (t0 / tEnd) * (graphRight - graphLeft);
        const response = (gain, rate, t) => {
          if (t < t0) return 0;
          return gain * (1 - Math.exp(-rate * (t - t0)));
        };
        const openResponse = (t) => response(mag / plantA, plantA, t);
        const feedbackResponse = (t) => {
          if (t < t0) return 0;
          const tau = t - t0;
          const damping = plantA + feedbackKp;
          const discriminant = Math.sqrt(Math.max(0, damping * damping - 4 * feedbackKi));
          const rootSlow = (-damping + discriminant) / 2;
          const rootFast = (-damping - discriminant) / 2;
          return mag * (Math.exp(rootSlow * tau) - Math.exp(rootFast * tau)) / discriminant;
        };
        const pathFromModel = (base, model) => {
          const points = [];
          for (let i = 0; i <= 56; i += 1) {
            const t = (i / 56) * tEnd;
            const px = graphLeft + (t / tEnd) * (graphRight - graphLeft);
            const py = base + yScale * model(t);
            points.push(`${px.toFixed(1)} ${py.toFixed(1)}`);
          }
          return `M ${points.join(" L ")}`;
        };
        const openFinal = openBase + yScale * openResponse(tEnd);
        const feedbackFinal = feedbackBase + yScale * feedbackResponse(tEnd);

        openLoop.setAttribute("d", pathFromModel(openBase, openResponse));
        feedback.setAttribute("d", pathFromModel(feedbackBase, feedbackResponse));

        if (line) {
          line.setAttribute("x1", x);
          line.setAttribute("x2", x);
        }
        if (arrow) {
          arrow.setAttribute("d", `M ${x - 12} 87 L ${x} 62 L ${x + 12} 87 Z`);
        }
        if (label) {
          label.setAttribute("x", Math.min(x + 26, 690));
          label.textContent = `disturbance at t = ${t0.toFixed(2)}`;
        }
        if (openEnd) {
          openEnd.setAttribute("cx", graphRight);
          openEnd.setAttribute("cy", openFinal);
        }
        if (feedbackEnd) {
          feedbackEnd.setAttribute("cx", graphRight);
          feedbackEnd.setAttribute("cy", feedbackFinal);
        }
        if (openLabel) {
          openLabel.setAttribute("y", Math.max(openFinal - 12, 142));
        }
        if (feedbackLabel) {
          feedbackLabel.setAttribute("x", Math.min(x + 90, 640));
          feedbackLabel.setAttribute("y", Math.min(feedbackFinal - 12, 252));
        }
      };

      time?.addEventListener("input", update);
      size?.addEventListener("input", update);
      update();
    }
  };

  const initChapterFourTaylorExplorer = () => {
    const panels = Array.from(document.querySelectorAll("[data-ch4-taylor-explorer]"));
    if (!panels.length) return;

    const factorial = (n) => {
      let value = 1;
      for (let i = 2; i <= n; i += 1) value *= i;
      return value;
    };
    const fmt = (value, digits = 3) => {
      if (!Number.isFinite(value)) return "undefined";
      if (Math.abs(value) < 1e-10) return "0";
      return value.toFixed(digits).replace(/\.?0+$/, "");
    };
    const termLabel = (k) => {
      if (k === 0) return "g(&omega;<sub>0</sub>)";
      if (k === 1) return "g'(&omega;<sub>0</sub>)(&omega; - &omega;<sub>0</sub>)";
      const primes = k === 2 ? "g''" : `g<sup>(${k})</sup>`;
      return `${primes}(&omega;<sub>0</sub>)(&omega; - &omega;<sub>0</sub>)<sup>${k}</sup>/${k}!`;
    };

    const dampedDerivatives = (w) => {
      const a = 0.65;
      const b = 2.4;
      const c = 0.27;
      let A = 1;
      let B = c;
      const derivatives = [1 - Math.exp(-a * w) * (A * Math.cos(b * w) + B * Math.sin(b * w))];
      for (let k = 1; k <= 5; k += 1) {
        const nextA = -a * A + b * B;
        const nextB = -b * A - a * B;
        A = nextA;
        B = nextB;
        derivatives.push(-Math.exp(-a * w) * (A * Math.cos(b * w) + B * Math.sin(b * w)));
      }
      return derivatives;
    };

    const functions = {
      damped: {
        label: "Figure 4.1 style damped curve",
        domain: [0, 8],
        yDomain: [-0.25, 1.45],
        defaultW0: 3.8,
        f: (w) => {
          const a = 0.65;
          const b = 2.4;
          const c = 0.27;
          return 1 - Math.exp(-a * w) * (Math.cos(b * w) + c * Math.sin(b * w));
        },
        derivatives: dampedDerivatives,
      },
      sin: {
        label: "sin(&omega;)",
        domain: [-6.3, 6.3],
        yDomain: [-1.35, 1.35],
        defaultW0: 1,
        f: (w) => Math.sin(w),
        derivatives: (w) => [
          Math.sin(w),
          Math.cos(w),
          -Math.sin(w),
          -Math.cos(w),
          Math.sin(w),
          Math.cos(w),
        ],
      },
      inverseSquare: {
        label: "1/&omega;<sup>2</sup>",
        domain: [0.5, 5],
        yDomain: [-0.2, 4.25],
        defaultW0: 2,
        f: (w) => 1 / (w * w),
        derivatives: (w) => {
          const out = [];
          for (let k = 0; k <= 5; k += 1) out.push(((-1) ** k) * factorial(k + 1) / (w ** (k + 2)));
          return out;
        },
      },
      exp: {
        label: "e<sup>&omega;</sup>",
        domain: [-2, 2],
        yDomain: [-0.5, 8.2],
        defaultW0: 0,
        f: (w) => Math.exp(w),
        derivatives: (w) => Array.from({ length: 6 }, () => Math.exp(w)),
      },
      log: {
        label: "ln(&omega;)",
        domain: [0.2, 5],
        yDomain: [-1.9, 1.8],
        defaultW0: 2,
        f: (w) => Math.log(w),
        derivatives: (w) => {
          const out = [Math.log(w)];
          for (let k = 1; k <= 5; k += 1) out.push(((-1) ** (k - 1)) * factorial(k - 1) / (w ** k));
          return out;
        },
      },
      reciprocal: {
        label: "1/(1 + &omega;)",
        domain: [-0.5, 5],
        yDomain: [-0.15, 2.25],
        defaultW0: 1,
        f: (w) => 1 / (1 + w),
        derivatives: (w) => {
          const out = [];
          for (let k = 0; k <= 5; k += 1) out.push(((-1) ** k) * factorial(k) / ((1 + w) ** (k + 1)));
          return out;
        },
      },
    };

    const draw = (panel) => {
      const selector = panel.querySelector("[data-ch4-taylor-function]");
      const w0Input = panel.querySelector("[data-ch4-taylor-w0]");
      const degreeInput = panel.querySelector("[data-ch4-taylor-degree]");
      const canvas = panel.querySelector("[data-ch4-taylor-canvas]");
      const ctx = canvas?.getContext?.("2d");
      if (!selector || !w0Input || !degreeInput || !canvas || !ctx) return;

      const definition = functions[selector.value] || functions.damped;
      const [domainMin, domainMax] = definition.domain;
      if (Number.parseFloat(w0Input.min) !== domainMin || Number.parseFloat(w0Input.max) !== domainMax) {
        w0Input.min = domainMin;
        w0Input.max = domainMax;
        const current = Number.parseFloat(w0Input.value);
        w0Input.value = Number.isFinite(current)
          ? Math.min(domainMax, Math.max(domainMin, current))
          : definition.defaultW0;
      }

      const w0 = Number.parseFloat(w0Input.value || String(definition.defaultW0));
      const degree = Number.parseInt(degreeInput.value || "1", 10);
      const derivatives = definition.derivatives(w0);
      const coefficients = derivatives.map((value, index) => value / factorial(index));
      const taylor = (w, maxDegree = degree) => {
        let value = 0;
        for (let k = 0; k <= maxDegree; k += 1) value += coefficients[k] * ((w - w0) ** k);
        return value;
      };
      const width = canvas.width;
      const height = canvas.height;
      const pad = { left: 72, right: 28, top: 32, bottom: 58 };
      const points = [];
      for (let i = 0; i <= 240; i += 1) {
        const w = domainMin + (i / 240) * (domainMax - domainMin);
        points.push({ w, actual: definition.f(w), approx: taylor(w), linear: taylor(w, Math.min(1, degree)) });
      }
      const [yMin, yMax] = definition.yDomain;
      const xMap = (w) => pad.left + ((w - domainMin) / (domainMax - domainMin)) * (width - pad.left - pad.right);
      const yMap = (y) => height - pad.bottom - ((y - yMin) / (yMax - yMin)) * (height - pad.top - pad.bottom);
      const drawCurve = (key, color, lineWidth, dash = []) => {
        ctx.save();
        ctx.beginPath();
        let drawing = false;
        points.forEach((point, index) => {
          const y = point[key];
          if (!Number.isFinite(y) || y < yMin || y > yMax) {
            drawing = false;
            return;
          }
          const px = xMap(point.w);
          const py = yMap(y);
          if (index === 0 || !drawing) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
          drawing = true;
        });
        ctx.strokeStyle = color;
        ctx.lineWidth = lineWidth;
        ctx.setLineDash(dash);
        ctx.stroke();
        ctx.restore();
      };

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);

      ctx.strokeStyle = "#dce8eb";
      ctx.lineWidth = 1;
      ctx.font = "13px sans-serif";
      ctx.fillStyle = "#526047";
      for (let i = 0; i <= 5; i += 1) {
        const w = domainMin + (i / 5) * (domainMax - domainMin);
        const x = xMap(w);
        ctx.beginPath();
        ctx.moveTo(x, pad.top);
        ctx.lineTo(x, height - pad.bottom);
        ctx.stroke();
        ctx.fillText(fmt(w, 2), x - 12, height - 34);
      }
      for (let i = 0; i <= 4; i += 1) {
        const y = yMin + (i / 4) * (yMax - yMin);
        const py = yMap(y);
        ctx.beginPath();
        ctx.moveTo(pad.left, py);
        ctx.lineTo(width - pad.right, py);
        ctx.stroke();
        ctx.fillText(fmt(y, 2), 16, py + 4);
      }

      ctx.strokeStyle = "#172d33";
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.moveTo(pad.left, height - pad.bottom);
      ctx.lineTo(width - pad.right, height - pad.bottom);
      ctx.moveTo(pad.left, pad.top);
      ctx.lineTo(pad.left, height - pad.bottom);
      ctx.stroke();
      ctx.fillStyle = "#172d33";
      ctx.font = "15px sans-serif";
      ctx.fillText("ω", width - 48, height - 16);
      ctx.fillText("g(ω)", pad.left + 8, pad.top - 8);

      drawCurve("actual", "#0e6d77", 3);
      if (degree > 1) drawCurve("linear", "#172d33", 2, [8, 6]);
      drawCurve("approx", degree === 1 ? "#d56b35" : "#9a5b12", 3, degree === 0 ? [5, 5] : []);

      const x0 = xMap(w0);
      const y0 = yMap(derivatives[0]);
      ctx.save();
      ctx.strokeStyle = "#172d33";
      ctx.lineWidth = 1.5;
      ctx.setLineDash([7, 6]);
      ctx.beginPath();
      ctx.moveTo(x0, pad.top);
      ctx.lineTo(x0, height - pad.bottom);
      ctx.stroke();
      ctx.setLineDash([2, 6]);
      ctx.beginPath();
      ctx.moveTo(pad.left, y0);
      ctx.lineTo(width - pad.right, y0);
      ctx.stroke();
      ctx.restore();

      ctx.fillStyle = "#d56b35";
      ctx.beginPath();
      ctx.arc(x0, y0, 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = "#172d33";
      ctx.font = "14px sans-serif";
      ctx.fillText("ω0", x0 - 16, height - 18);
      ctx.save();
      const pointLabelY = Math.min(height - pad.bottom - 12, Math.max(pad.top + 18, y0 - 10));
      ctx.fillStyle = "rgba(251, 253, 254, 0.88)";
      ctx.fillRect(pad.left + 10, pointLabelY - 16, 54, 20);
      ctx.fillStyle = "#172d33";
      ctx.fillText("g(ω0)", pad.left + 14, pointLabelY);
      ctx.restore();

      ctx.fillStyle = "#0e6d77";
      ctx.fillRect(width - 230, 22, 18, 4);
      ctx.fillStyle = "#172d33";
      ctx.fillText("nonlinear g(ω)", width - 204, 28);
      ctx.fillStyle = degree === 1 ? "#d56b35" : "#9a5b12";
      ctx.fillRect(width - 230, 46, 18, 4);
      ctx.fillStyle = "#172d33";
      ctx.fillText(`Taylor degree ${degree}`, width - 204, 52);
      if (degree > 1) {
        ctx.fillStyle = "#172d33";
        ctx.fillRect(width - 230, 70, 18, 4);
        ctx.fillText("first-order linearization", width - 204, 76);
      }
      if (degree === 1) {
        const badgeX = pad.left + 96;
        const badgeY = pad.top + 10;
        ctx.fillStyle = "#fff8e8";
        ctx.strokeStyle = "#d56b35";
        ctx.lineWidth = 1.4;
        ctx.fillRect(badgeX, badgeY, 188, 38);
        ctx.strokeRect(badgeX, badgeY, 188, 38);
        ctx.fillStyle = "#57411d";
        ctx.font = "700 14px sans-serif";
        ctx.fillText("Linearized solution", badgeX + 14, badgeY + 24);
      }

      const symbolicTerms = [];
      for (let k = 0; k <= degree; k += 1) symbolicTerms.push(termLabel(k));
      const numericTerms = [];
      for (let k = 0; k <= degree; k += 1) {
        const coefficient = coefficients[k];
        const magnitude = Math.abs(coefficient);
        const base = k === 0
          ? fmt(magnitude, 4)
          : `${fmt(magnitude, 4)}(&omega; - ${fmt(w0, 3)})${k > 1 ? `<sup>${k}</sup>` : ""}`;
        if (k === 0) numericTerms.push(coefficient < 0 ? `-${base}` : base);
        else numericTerms.push(`${coefficient < 0 ? " - " : " + "}${base}`);
      }
      const status = degree === 0
        ? "constant approximation only"
        : degree === 1
          ? "linearized solution: constant plus first derivative term"
          : "higher-order Taylor approximation: nonlinear terms are included";

      panel.querySelector("[data-ch4-taylor-w0-value]").innerHTML = fmt(w0, 3);
      panel.querySelector("[data-ch4-taylor-degree-value]").innerHTML = String(degree);
      panel.querySelector("[data-ch4-taylor-status]").innerHTML = status;
      panel.querySelector("[data-ch4-taylor-symbolic]").innerHTML = `g(&omega;) &asymp; ${symbolicTerms.join(" + ")}`;
      panel.querySelector("[data-ch4-taylor-numeric]").innerHTML = `g(&omega;) &asymp; ${numericTerms.join("")}`;
      panel.querySelector("[data-ch4-taylor-point]").innerHTML = `For ${definition.label}, g(&omega;<sub>0</sub>) = ${fmt(derivatives[0], 4)} and g'(&omega;<sub>0</sub>) = ${fmt(derivatives[1], 4)}.`;
    };

    for (const panel of panels) {
      const selector = panel.querySelector("[data-ch4-taylor-function]");
      const w0Input = panel.querySelector("[data-ch4-taylor-w0]");
      const degreeInput = panel.querySelector("[data-ch4-taylor-degree]");
      selector?.addEventListener("change", () => {
        const definition = functions[selector.value] || functions.damped;
        if (w0Input) {
          w0Input.min = definition.domain[0];
          w0Input.max = definition.domain[1];
          w0Input.value = definition.defaultW0;
        }
        draw(panel);
      });
      w0Input?.addEventListener("input", () => draw(panel));
      degreeInput?.addEventListener("input", () => draw(panel));
      draw(panel);
    }
  };

  const initChapterNineActivities = () => {
	    const fvtCases = {
      stableStep: {
        transfer: "Y(s) = 1 / (s(s + 2))",
        sy: "sY(s) = 1 / (s + 2)",
        poles: "s = -2",
        checks: [
          "No right-half-plane poles.",
          "No nonzero imaginary-axis poles.",
          "No pole remains at the origin in sY(s).",
        ],
        answer: "valid",
        limit: "lim sY(s) = lim 1 / (s + 2) = 0.5",
        status: "Valid use of the final value theorem.",
        result: "The response approaches 0.5 because the poles of sY(s) are in the left half plane.",
        path: "M 50 230 C 150 218, 215 165, 300 125 C 415 70, 560 82, 690 82",
        finalLine: 82,
        label: "final value = 0.5",
      },
      decaying: {
        transfer: "Y(s) = 1 / (s + 2)",
        sy: "sY(s) = s / (s + 2)",
        poles: "s = -2",
        checks: [
          "No right-half-plane poles.",
          "No nonzero imaginary-axis poles.",
          "The response decays to a finite value.",
        ],
        answer: "valid",
        limit: "lim sY(s) = lim s / (s + 2) = 0",
        status: "Valid use of the final value theorem.",
        result: "The time response decays to zero.",
        path: "M 50 70 C 145 88, 220 138, 330 185 C 455 236, 560 243, 690 245",
        finalLine: 245,
        label: "final value = 0",
      },
      unstable: {
        transfer: "Y(s) = 1 / (s(s - 1))",
        sy: "sY(s) = 1 / (s - 1)",
        poles: "s = 1",
        checks: [
          "There is a right-half-plane pole.",
          "The algebraic limit exists, but the time response does not settle.",
          "The final value theorem is not valid for this case.",
        ],
        answer: "unstable",
        limit: "lim sY(s) = lim 1 / (s - 1) = -1",
        status: "Do not use the algebraic limit as the final value.",
        result: "The response is unstable because sY(s) has a right-half-plane pole at s = 1.",
        path: "M 50 230 C 160 222, 250 205, 355 178 C 490 143, 590 86, 690 38",
        finalLine: 245,
        label: "no finite final value",
      },
      oscillatory: {
        transfer: "Y(s) = 1 / (s^2 + 1)",
        sy: "sY(s) = s / (s^2 + 1)",
        poles: "s = +j, -j",
        checks: [
          "There are nonzero imaginary-axis poles.",
          "The time response keeps oscillating.",
          "The final value theorem is not valid for this case.",
        ],
        answer: "oscillatory",
        limit: "lim sY(s) = lim s / (s^2 + 1) = 0",
        status: "Do not use the algebraic limit as the final value.",
        result: "The response keeps oscillating because sY(s) has imaginary-axis poles at s = +/-j.",
        path: "M 50 145 C 90 75, 135 75, 175 145 C 215 215, 260 215, 300 145 C 340 75, 385 75, 425 145 C 465 215, 510 215, 550 145 C 590 75, 635 75, 690 145",
        finalLine: 145,
        label: "oscillates; no final value",
      },
      unbounded: {
        transfer: "Y(s) = 1 / s^2",
        sy: "sY(s) = 1 / s",
        poles: "s = 0",
        checks: [
          "A pole remains at the origin in sY(s).",
          "The response is a ramp, so the final value is unbounded.",
          "The final value theorem is not valid for this case.",
        ],
        answer: "unbounded",
        limit: "lim sY(s) = lim 1 / s = infinity",
        status: "Do not use the final value theorem because the final value is not finite.",
        result: "The response grows without bound, so there is no finite steady-state value to compute.",
        path: "M 50 245 L 690 45",
        finalLine: 245,
        label: "unbounded ramp",
      },
    };
    for (const explorer of document.querySelectorAll("[data-ch9-fvt-explorer]")) {
      const select = explorer.querySelector("[data-ch9-fvt-case]");
      const transfer = explorer.querySelector("[data-ch9-fvt-transfer]");
      const sy = explorer.querySelector("[data-ch9-fvt-sy]");
      const poles = explorer.querySelector("[data-ch9-fvt-poles]");
      const checklist = explorer.querySelector("[data-ch9-fvt-checklist]");
      const limit = explorer.querySelector("[data-ch9-fvt-limit]");
      const status = explorer.querySelector("[data-ch9-fvt-status]");
      const result = explorer.querySelector("[data-ch9-fvt-result]");
      const results = explorer.querySelector("[data-ch9-fvt-results]");
      const feedback = explorer.querySelector("[data-ch9-fvt-feedback]");
      const check = explorer.querySelector("[data-ch9-fvt-check]");
      const reset = explorer.querySelector("[data-ch9-fvt-reset]");
      const responsePath = explorer.querySelector("[data-ch9-fvt-response-path]");
      const finalLine = explorer.querySelector("[data-ch9-fvt-final-line]");
      const plotLabel = explorer.querySelector("[data-ch9-fvt-plot-label]");
      const predictions = Array.from(explorer.querySelectorAll('input[name="pid-ch9-fvt-prediction"]'));

      const hidePlotAnswer = () => {
        responsePath?.setAttribute("d", "");
        if (plotLabel) plotLabel.textContent = "";
        if (finalLine) finalLine.style.display = "none";
      };

      const showPlotAnswer = (item) => {
        responsePath?.setAttribute("d", item.path);
        if (plotLabel) plotLabel.textContent = item.label;
        if (finalLine) {
          finalLine.setAttribute("y1", item.finalLine);
          finalLine.setAttribute("y2", item.finalLine);
          finalLine.style.display = "";
        }
      };

      const showFeedback = (ok, message) => {
        if (!feedback) return;
        feedback.classList.remove("is-correct", "is-wrong");
        feedback.classList.add(ok ? "is-correct" : "is-wrong");
        feedback.textContent = message;
      };

      const clearFeedback = () => {
        if (feedback) {
          feedback.textContent = "";
          feedback.classList.remove("is-correct", "is-wrong");
        }
        if (results) results.hidden = true;
        hidePlotAnswer();
      };

      const update = () => {
        const item = fvtCases[select?.value] || fvtCases.stableStep;
        if (transfer) transfer.textContent = item.transfer;
        if (sy) sy.textContent = item.sy;
        if (poles) poles.textContent = `Poles of sY(s): ${item.poles}`;
        if (checklist) {
          checklist.innerHTML = "";
          for (const line of item.checks) {
            const entry = document.createElement("li");
            entry.textContent = line;
            checklist.append(entry);
          }
        }
        if (limit) limit.textContent = item.limit;
        if (status) status.textContent = item.status;
        if (result) result.textContent = item.result;
        for (const prediction of predictions) prediction.checked = false;
        clearFeedback();
      };
      select?.addEventListener("change", update);
      for (const prediction of predictions) {
        prediction.addEventListener("change", clearFeedback);
      }
      check?.addEventListener("click", () => {
        const item = fvtCases[select?.value] || fvtCases.stableStep;
        const chosen = predictions.find((prediction) => prediction.checked)?.value;
        if (!chosen) {
          showFeedback(false, "Make a prediction first, then click Check.");
          return;
        }
        const ok = chosen === item.answer;
        showFeedback(
          ok,
          ok
            ? "Correct. Now compare your prediction with the pole check and algebraic limit below."
            : "Not quite. Compare the poles of sY(s) with the final value theorem conditions below.",
        );
        if (results) results.hidden = false;
        showPlotAnswer(item);
      });
      reset?.addEventListener("click", () => {
        for (const prediction of predictions) prediction.checked = false;
        clearFeedback();
      });
      update();
    }

    const steadyStateErrorHTML = (text) => text.replace(/e_ss/g, "e<sub>ss</sub>");
    const setSteadyStateErrorSVGText = (element, text) => {
      if (!element) return;
      element.textContent = "";
      const parts = text.split("e_ss");
      if (parts.length === 1) {
        element.textContent = text;
        return;
      }
      element.append(document.createTextNode(parts[0]));
      const subscript = document.createElementNS("http://www.w3.org/2000/svg", "tspan");
      subscript.setAttribute("baseline-shift", "sub");
      subscript.setAttribute("font-size", "12");
      subscript.textContent = "ss";
      element.append(document.createTextNode("e"), subscript, document.createTextNode(parts.slice(1).join("e_ss")));
    };

    const inputOrders = {
      step: { order: 0, label: "step", transform: "1/s", constant: "M_p", constantHTML: "M<sub>p</sub>" },
      ramp: { order: 1, label: "ramp", transform: "1/s^2", constant: "M_v", constantHTML: "M<sub>v</sub>" },
      parabola: { order: 2, label: "parabola", transform: "1/s^3", constant: "M_a", constantHTML: "M<sub>a</sub>" },
    };

    const makeReferencePath = (order) => {
      if (order === 0) return "M 50 80 L 690 80";
      if (order === 1) return "M 50 235 L 690 80";
      return "M 50 245 C 210 240, 480 160, 690 70";
    };

    const getReferenceEndY = (order) => {
      if (order === 0) return 80;
      if (order === 1) return 80;
      return 70;
    };

    const classifyReferenceTrackingError = (type, order) => {
      if (type >= order + 1) return "zero";
      if (type === order) return "finite";
      return "infinite";
    };

    const makeTrackingVisualization = (type, order, errorPixels) => {
      const referenceEndY = getReferenceEndY(order);
      const errorClass = classifyReferenceTrackingError(type, order);
      let responsePath;
      let errorPath;
      let responseEndY = referenceEndY;
      let errorLineY = 145;
      let errorLabel = "e_ss = 0";

      if (errorClass === "zero") {
        if (order === 0) responsePath = "M 50 230 C 150 220, 230 115, 360 84 C 470 78, 570 80, 690 80";
        else if (order === 1) responsePath = "M 50 245 C 155 238, 245 205, 365 162 C 500 114, 610 88, 690 80";
        else responsePath = "M 50 245 C 220 238, 480 165, 690 70";
        errorPath = "M 50 62 C 145 80, 220 122, 335 140 C 450 150, 570 145, 690 145";
      } else if (errorClass === "finite") {
        responseEndY = referenceEndY + errorPixels;
        errorLineY = 145 - errorPixels;
        errorLabel = "finite e_ss";
        if (order === 0) {
          responsePath = `M 50 230 C 150 220, 230 ${responseEndY + 40}, 360 ${responseEndY + 8} C 470 ${responseEndY - 4}, 570 ${responseEndY}, 690 ${responseEndY}`;
        } else if (order === 1) {
          responsePath = `M 50 245 C 160 246, 270 ${210 + errorPixels * 0.25}, 390 ${165 + errorPixels * 0.45} C 510 ${126 + errorPixels * 0.7}, 610 ${96 + errorPixels}, 690 ${responseEndY}`;
        } else {
          responsePath = `M 50 245 C 220 245, 480 ${175 + errorPixels * 0.55}, 690 ${responseEndY}`;
        }
        errorPath = `M 50 145 C 145 140, 235 ${errorLineY + 20}, 350 ${errorLineY + 5} C 470 ${errorLineY - 4}, 575 ${errorLineY}, 690 ${errorLineY}`;
      } else {
        responseEndY = order === 1 ? 205 : 190;
        errorLineY = 45;
        errorLabel = "error grows without bound";
        if (order === 1) {
          responsePath = "M 50 245 C 170 244, 285 235, 410 224 C 535 214, 625 207, 690 205";
          errorPath = "M 50 145 C 160 135, 260 112, 390 88 C 515 63, 610 50, 690 42";
        } else {
          responsePath = "M 50 245 C 210 247, 430 232, 690 190";
          errorPath = "M 50 145 C 155 140, 260 118, 390 86 C 520 55, 615 38, 690 34";
        }
      }

      return {
        responseEndY,
        responsePath,
        errorPath,
        referenceEndY,
        errorLineY,
        errorLabel,
        showFiniteGap: errorClass !== "zero",
        showErrorFinalLine: errorClass !== "infinite",
      };
    };

    for (const explorer of document.querySelectorAll("[data-ch9-reference-explorer]")) {
      const typeSelect = explorer.querySelector("[data-ch9-reference-type]");
      const inputSelect = explorer.querySelector("[data-ch9-reference-input]");
      const gainInput = explorer.querySelector("[data-ch9-reference-gain]");
      const gainValue = explorer.querySelector("[data-ch9-reference-gain-value]");
      const result = explorer.querySelector("[data-ch9-reference-result]");
      const formula = explorer.querySelector("[data-ch9-reference-formula]");
      const errorRule = explorer.querySelector("[data-ch9-reference-error-rule]");
      const referencePath = explorer.querySelector("[data-ch9-reference-path]");
      const responsePath = explorer.querySelector("[data-ch9-response-path]");
      const errorPath = explorer.querySelector("[data-ch9-error-path]");
      const errorFinalLine = explorer.querySelector("[data-ch9-error-final-line]");
      const errorPlotLabel = explorer.querySelector("[data-ch9-error-plot-label]");
      const errorOffsetLine = explorer.querySelector("[data-ch9-error-offset-line]");
      const responseErrorLabel = explorer.querySelector("[data-ch9-response-error-label]");
      const badge = explorer.querySelector("[data-ch9-reference-badge]");
      const update = () => {
        const type = Number.parseInt(typeSelect?.value || "0", 10);
        const input = inputOrders[inputSelect?.value] || inputOrders.step;
        const gain = Math.max(Number.parseFloat(gainInput?.value || "4"), 0.1);
        const finiteErrorValue = input.order === 0 ? 1 / (1 + gain) : 1 / gain;
        const errorPixels = Math.min(92, 18 + 78 * Math.min(finiteErrorValue, 1));
        const errorClass = classifyReferenceTrackingError(type, input.order);
        const visual = makeTrackingVisualization(type, input.order, errorPixels);
        let resultText;
        let formulaText;
        let errorRuleText;
        let errorSummary;
        if (errorClass === "zero") {
          resultText = `Type ${type} gives zero steady-state error for a ${input.label}: e_ss = 0.`;
          formulaText = `Because the system has more free integrators than the ${input.transform} input requires, e_ss = 0.`;
          errorRuleText = `The ${input.constantHTML} slider does not change the steady-state result in this case because the extra free integrator drives e_ss to zero.`;
          errorSummary = "e_ss = 0";
        } else if (errorClass === "finite") {
          resultText = `Type ${type} gives finite steady-state error for a ${input.label}: e_ss = ${finiteErrorValue.toFixed(3)}.`;
          formulaText =
            input.order === 0
              ? `Using ${input.constantHTML} = ${gain.toFixed(2)}, e_ss = 1 / (1 + ${input.constantHTML}).`
              : `Using ${input.constantHTML} = ${gain.toFixed(2)}, e_ss = 1 / ${input.constantHTML}.`;
          errorRuleText =
            input.order === 0
              ? `Move the slider to change ${input.constantHTML}. Larger ${input.constantHTML} reduces the final step error 1 / (1 + ${input.constantHTML}) and visibly closes the gap.`
              : `Move the slider to change ${input.constantHTML}. Larger ${input.constantHTML} reduces the final ${input.label} error 1 / ${input.constantHTML} and lowers the error plot.`;
          errorSummary = `e_ss = ${finiteErrorValue.toFixed(3)}`;
        } else {
          resultText = `Type ${type} cannot keep up with a ${input.label}; e_ss is infinite.`;
          formulaText = `The ${input.transform} input has higher order than the available free integrators.`;
          errorRuleText = `Changing ${input.constantHTML} cannot fix the missing integrator. The error grows without bound because the input order is higher than the system type.`;
          errorSummary = "e_ss is infinite";
        }
        if (gainValue) gainValue.innerHTML = `${input.constantHTML} = ${gain.toFixed(2)}`;
        if (result) result.innerHTML = steadyStateErrorHTML(resultText);
        if (formula) formula.innerHTML = steadyStateErrorHTML(formulaText);
        if (errorRule) errorRule.innerHTML = steadyStateErrorHTML(errorRuleText);
        if (badge) badge.innerHTML = steadyStateErrorHTML(`reference: ${input.label}, type: ${type}, ${errorSummary}`);
        referencePath?.setAttribute("d", makeReferencePath(input.order));
        responsePath?.setAttribute("d", visual.responsePath);
        errorPath?.setAttribute("d", visual.errorPath);
        if (errorFinalLine) {
          errorFinalLine.setAttribute("y1", visual.errorLineY);
          errorFinalLine.setAttribute("y2", visual.errorLineY);
          errorFinalLine.style.display = visual.showErrorFinalLine ? "" : "none";
        }
        setSteadyStateErrorSVGText(errorPlotLabel, errorSummary);
        if (errorOffsetLine) {
          errorOffsetLine.setAttribute("y1", visual.referenceEndY);
          errorOffsetLine.setAttribute("y2", visual.responseEndY);
          errorOffsetLine.style.display = visual.showFiniteGap ? "" : "none";
        }
        if (responseErrorLabel) {
          setSteadyStateErrorSVGText(responseErrorLabel, visual.showFiniteGap ? visual.errorLabel : "");
          responseErrorLabel.setAttribute("y", Math.min(Math.max((visual.referenceEndY + visual.responseEndY) / 2, 58), 222));
        }
      };
      typeSelect?.addEventListener("change", update);
      inputSelect?.addEventListener("change", update);
      gainInput?.addEventListener("input", update);
      update();
    }

    const disturbanceCopy = {
      inputP: {
        path: "Input disturbance through P(s) / (1 + P(s)C(s))",
        type: "With proportional control, the input-disturbance type is type 0 in the chapter example.",
        effect: "A constant input disturbance leaves finite steady-state error.",
        note: "d_in enters before the plant, so P(s) remains in the numerator of the error path.",
      },
      inputPI: {
        path: "Input disturbance through P(s) / (1 + P(s)C(s))",
        type: "With PI or PID control, the controller contributes one free integrator.",
        effect: "A constant input disturbance is rejected with zero steady-state error; a ramp disturbance leaves finite error.",
        note: "The signal enters at the same plant-input location, but the controller integrator changes the low-frequency error path.",
      },
      outputP: {
        path: "Output disturbance through 1 / (1 + P(s)C(s))",
        type: "This path has the same type dependence as reference tracking.",
        effect: "The relevant integrators are in the open-loop product P(s)C(s).",
        note: "d_out enters after the plant, so it uses the same denominator path as reference tracking.",
      },
      noisePI: {
        path: "Sensor noise through 1 / (1 + P(s)C(s))",
        type: "The same algebraic path appears, but sensor noise is usually high frequency.",
        effect: "Integral action is not a high-frequency noise solution; filtering and bandwidth choices matter.",
        note: "Sensor noise enters the measured feedback path before the negative input of the summing junction.",
      },
    };

    for (const explorer of document.querySelectorAll("[data-ch9-disturbance-explorer]")) {
      const controls = Array.from(explorer.querySelectorAll("[data-ch9-disturbance-case]"));
      const path = explorer.querySelector("[data-ch9-disturbance-path]");
      const type = explorer.querySelector("[data-ch9-disturbance-type]");
      const effect = explorer.querySelector("[data-ch9-disturbance-effect]");
      const note = explorer.querySelector("[data-ch9-disturbance-diagram-note]");
      const highlights = Array.from(explorer.querySelectorAll("[data-ch9-disturbance-path-highlight]"));
      const blockHighlights = Array.from(explorer.querySelectorAll("[data-ch9-disturbance-block-highlight]"));
      const update = (value) => {
        const item = disturbanceCopy[value] || disturbanceCopy.inputP;
        for (const control of controls) control.classList.toggle("is-active", control.dataset.ch9DisturbanceCase === value);
        for (const highlight of highlights) {
          const cases = (highlight.dataset.ch9DisturbancePathHighlight || "").split(/\s+/);
          highlight.classList.toggle("is-active", cases.includes(value));
        }
        for (const highlight of blockHighlights) {
          const cases = (highlight.dataset.ch9DisturbanceBlockHighlight || "").split(/\s+/);
          highlight.classList.toggle("is-active", cases.includes(value));
        }
        if (path) path.textContent = item.path;
        if (type) type.textContent = item.type;
        if (effect) effect.textContent = item.effect;
        if (note) note.textContent = item.note;
      };
      for (const control of controls) {
        control.addEventListener("click", () => update(control.dataset.ch9DisturbanceCase));
      }
      update(controls.find((control) => control.classList.contains("is-active"))?.dataset.ch9DisturbanceCase || "inputP");
    }

    const normalizeFinalValue = (value) => {
      const normalized = String(value || "").trim().toLowerCase();
      if (["inf", "infinite", "infinity", "∞"].includes(normalized)) return "infinity";
      if (["zero", "0.0", "0.00", "+0", "-0"].includes(normalized)) return "0";
      if (["one", "1.0", "1.00", "+1"].includes(normalized)) return "1";
      return normalized;
    };

    for (const table of document.querySelectorAll("[data-ch9-final-value-table]")) {
      const inputs = Array.from(table.querySelectorAll("[data-ch9-final-answer]"));
      const check = table.querySelector("[data-ch9-final-check]");
      const reset = table.querySelector("[data-ch9-final-reset]");
      const feedback = table.querySelector("[data-ch9-final-feedback]");
      const setFeedback = (ok, text) => {
        if (!feedback) return;
        feedback.classList.remove("is-correct", "is-wrong");
        feedback.classList.add(ok ? "is-correct" : "is-wrong");
        feedback.textContent = text;
      };
      check?.addEventListener("click", () => {
        let correct = 0;
        for (const input of inputs) {
          const expected = normalizeFinalValue(input.dataset.ch9FinalAnswer);
          const actual = normalizeFinalValue(input.value);
          const ok = actual === expected;
          input.classList.toggle("is-correct", ok);
          input.classList.toggle("is-wrong", !ok);
          if (ok) correct += 1;
        }
        setFeedback(
          correct === inputs.length,
          correct === inputs.length
            ? "Correct. All final values match the low-frequency type of the three paths."
            : `${correct} of ${inputs.length} entries are correct. Recheck the path type before applying the final value theorem.`,
        );
      });
      reset?.addEventListener("click", () => {
        for (const input of inputs) {
          input.value = "";
          input.classList.remove("is-correct", "is-wrong");
        }
        if (feedback) {
          feedback.textContent = "";
          feedback.classList.remove("is-correct", "is-wrong");
        }
      });
      for (const input of inputs) {
        input.addEventListener("input", () => {
          input.classList.remove("is-correct", "is-wrong");
          if (feedback) {
            feedback.textContent = "";
            feedback.classList.remove("is-correct", "is-wrong");
          }
        });
      }
    }
  };

  const initAdvancedChapterActivities = () => {
    const fmt = (value, digits = 2) => {
      const number = Number(value);
      if (!Number.isFinite(number)) return String(value);
      return number.toFixed(digits).replace(/\.?0+$/, "");
    };
    const setText = (root, key, value) => {
      const target = root.querySelector(`[data-output="${key}"]`);
      if (target) target.textContent = value;
    };
    const setHTML = (root, key, value) => {
      const target = root.querySelector(`[data-output="${key}"]`);
      if (target) target.innerHTML = value;
    };
    const setAttr = (root, key, name, value) => {
      const target = root.querySelector(`[data-figure="${key}"]`);
      if (target) target.setAttribute(name, value);
    };
    const val = (root, key, fallback = 0) => {
      const input = root.querySelector(`[data-input="${key}"]`);
      const number = Number.parseFloat(input?.value ?? "");
      return Number.isFinite(number) ? number : fallback;
    };
    const selectValue = (root, key, fallback = "") => {
      const input = root.querySelector(`[data-select="${key}"]`);
      return input?.value || fallback;
    };
    const drawCanvasRoundedRect = (ctx, x, y, width, height, radius = 8) => {
      ctx.beginPath();
      if (ctx.roundRect) {
        ctx.roundRect(x, y, width, height, radius);
      } else {
        ctx.rect(x, y, width, height);
      }
      ctx.fill();
      ctx.stroke();
    };
    const drawFirstOrderStepCanvas = (activity, p, amplitude) => {
      const canvas = activity.querySelector("[data-plot='first-order-step']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      const pad = { left: 68, right: 32, top: 30, bottom: 58 };
      const tEnd = Math.max(5, 5 / Math.max(p, 0.2));
      const yMax = amplitude * 1.08;
      const xMap = (t) => pad.left + (t / tEnd) * (width - pad.left - pad.right);
      const yMap = (y) => height - pad.bottom - (y / yMax) * (height - pad.top - pad.bottom);
      const color = "#0e6d77";

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);
      ctx.strokeStyle = "#dce8eb";
      ctx.lineWidth = 1;
      ctx.font = "13px sans-serif";
      ctx.fillStyle = "#526047";

      for (let i = 0; i <= 5; i += 1) {
        const t = (i / 5) * tEnd;
        const x = xMap(t);
        ctx.beginPath();
        ctx.moveTo(x, pad.top);
        ctx.lineTo(x, height - pad.bottom);
        ctx.stroke();
        ctx.fillText(fmt(t, 1), x - 10, height - 34);
      }
      for (let i = 0; i <= 4; i += 1) {
        const y = (i / 4) * yMax;
        const py = yMap(y);
        ctx.beginPath();
        ctx.moveTo(pad.left, py);
        ctx.lineTo(width - pad.right, py);
        ctx.stroke();
        ctx.fillText(fmt(y, 1), 20, py + 4);
      }

      ctx.strokeStyle = "#172d33";
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.moveTo(pad.left, height - pad.bottom);
      ctx.lineTo(width - pad.right, height - pad.bottom);
      ctx.moveTo(pad.left, pad.top);
      ctx.lineTo(pad.left, height - pad.bottom);
      ctx.stroke();
	      ctx.fillStyle = "#172d33";
	      ctx.font = "15px sans-serif";
	      ctx.fillText("t", width - 48, height - 18);
	      ctx.fillText("y(t)", pad.left + 12, 22);

      const riseTime = Math.log(10) / p;
      const riseX = xMap(riseTime);
      ctx.save();
      ctx.strokeStyle = "#172d33";
      ctx.lineWidth = 1.4;
      ctx.setLineDash([7, 6]);
      ctx.beginPath();
      ctx.moveTo(riseX, pad.top);
      ctx.lineTo(riseX, height - pad.bottom);
      ctx.stroke();
      ctx.restore();
      ctx.fillStyle = "#172d33";
      ctx.font = "13px sans-serif";
      ctx.fillText("90% rise time", Math.min(riseX + 8, width - 138), pad.top + 18);

      ctx.beginPath();
      for (let i = 0; i <= 240; i += 1) {
        const t = (i / 240) * tEnd;
        const y = amplitude * (1 - Math.exp(-p * t));
        const x = xMap(t);
        const py = yMap(y);
        if (i === 0) ctx.moveTo(x, py);
        else ctx.lineTo(x, py);
      }
      ctx.strokeStyle = color;
      ctx.lineWidth = 3;
      ctx.stroke();

      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.6;
      ctx.setLineDash([4, 5]);
      ctx.beginPath();
      ctx.moveTo(pad.left, yMap(amplitude));
      ctx.lineTo(width - pad.right, yMap(amplitude));
      ctx.stroke();
      ctx.restore();

      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(riseX, yMap(0.9 * amplitude), 5, 0, Math.PI * 2);
      ctx.fill();

      const tangentT = Math.min(tEnd * 0.18, 1 / p);
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      ctx.moveTo(xMap(0), yMap(0));
      ctx.lineTo(xMap(tangentT), yMap(amplitude * p * tangentT));
      ctx.stroke();

      const legendX = width - 190;
      ctx.fillStyle = color;
      ctx.fillRect(legendX, 28, 18, 5);
      ctx.fillStyle = "#172d33";
      ctx.fillText(`A = ${fmt(amplitude, 1)}, slope = ${fmt(amplitude * p, 2)}`, legendX + 26, 34);
    };
    const drawDirtyDerivativeCanvas = (activity, sigma) => {
      const canvas = activity.querySelector("[data-plot='dirty-derivative']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;

      const width = canvas.width;
      const height = canvas.height;
      const pad = { left: 72, right: 36, top: 42, bottom: 52 };
      const ts = 0.02;
      const samples = 251;
      const tEnd = ts * (samples - 1);
      const beta = (2 * sigma - ts) / (2 * sigma + ts);
      const gain = 2 / (2 * sigma + ts);
      const plotLeft = pad.left;
      const plotRight = width - pad.right;
      const signalPlot = { top: pad.top, bottom: 162 };
      const derivativePlot = { top: 214, bottom: height - pad.bottom };
      const tMap = (t) => plotLeft + (t / tEnd) * (plotRight - plotLeft);
      const makeYMap = (top, bottom, yMin, yMax) => (y) => {
        const clipped = Math.max(yMin, Math.min(yMax, y));
        return bottom - ((clipped - yMin) / (yMax - yMin)) * (bottom - top);
      };
      const signalYMap = makeYMap(signalPlot.top, signalPlot.bottom, -1.5, 1.5);
      const derivativeYLimit = 18;
      const derivativeYMap = makeYMap(derivativePlot.top, derivativePlot.bottom, -derivativeYLimit, derivativeYLimit);
      const signal = [];
      const rawDerivative = [0];
      const dirtyDerivative = [0];
      const trueDerivative = [];

      for (let i = 0; i < samples; i += 1) {
        const t = i * ts;
        const smooth = Math.sin(2.2 * t) + 0.25 * Math.sin(5.4 * t + 0.6);
        const noise = 0.11 * Math.sin(73 * t + 0.8) + 0.07 * Math.sin(131 * t + 1.7);
        signal.push(smooth + noise);
        trueDerivative.push(2.2 * Math.cos(2.2 * t) + 0.25 * 5.4 * Math.cos(5.4 * t + 0.6));
      }
      for (let i = 1; i < samples; i += 1) {
        const delta = signal[i] - signal[i - 1];
        rawDerivative.push(delta / ts);
        dirtyDerivative.push(beta * dirtyDerivative[i - 1] + gain * delta);
      }

      const drawPath = (values, yMap, color, lineWidth = 2.5, dash = []) => {
        ctx.save();
        ctx.beginPath();
        ctx.setLineDash(dash);
        values.forEach((value, index) => {
          const x = tMap(index * ts);
          const y = yMap(value);
          if (index === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.strokeStyle = color;
        ctx.lineWidth = lineWidth;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.stroke();
        ctx.restore();
      };
      const drawRoundedRect = (x, y, w, h, r) => {
        if (ctx.roundRect) {
          ctx.beginPath();
          ctx.roundRect(x, y, w, h, r);
          ctx.fill();
          ctx.stroke();
        } else {
          ctx.fillRect(x, y, w, h);
          ctx.strokeRect(x, y, w, h);
        }
      };

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);
      ctx.font = "13px sans-serif";
      ctx.lineWidth = 1;

      const drawPanel = (panel, yMap, yTicks) => {
        ctx.strokeStyle = "#dce8eb";
        ctx.lineWidth = 1;
        for (let i = 0; i <= 5; i += 1) {
          const x = plotLeft + (i / 5) * (plotRight - plotLeft);
          ctx.beginPath();
          ctx.moveTo(x, panel.top);
          ctx.lineTo(x, panel.bottom);
          ctx.stroke();
        }
        for (const y of yTicks) {
          ctx.beginPath();
          ctx.moveTo(plotLeft, yMap(y));
          ctx.lineTo(plotRight, yMap(y));
          ctx.stroke();
        }
        ctx.strokeStyle = "#172d33";
        ctx.lineWidth = 2;
        ctx.strokeRect(plotLeft, panel.top, plotRight - plotLeft, panel.bottom - panel.top);
      };

      drawPanel(signalPlot, signalYMap, [-1.5, -0.75, 0, 0.75, 1.5]);
      drawPanel(derivativePlot, derivativeYMap, [-18, -9, 0, 9, 18]);

      const smoothSignal = signal.map((_, index) => {
        const t = index * ts;
        return Math.sin(2.2 * t) + 0.25 * Math.sin(5.4 * t + 0.6);
      });
      drawPath(smoothSignal, signalYMap, "#64748b", 2.6, [7, 5]);
      drawPath(signal, signalYMap, "#526047", 2.2);
      drawPath(trueDerivative, derivativeYMap, "#7b5ea7", 2.4, [7, 5]);
      drawPath(rawDerivative, derivativeYMap, "#d56b35", 1.8);
      drawPath(dirtyDerivative, derivativeYMap, "#0e6d77", 3);

      ctx.fillStyle = "#244b54";
      ctx.font = "700 14px sans-serif";
      ctx.fillText("Noisy input signal v(t)", plotLeft, signalPlot.top - 16);
      ctx.fillText("Derivative estimates from noisy v(t)", plotLeft, derivativePlot.top - 16);
      ctx.font = "13px sans-serif";
      ctx.fillStyle = "#526047";
      ctx.fillText("time (s)", plotRight - 54, height - 26);
      ctx.fillText("+1.5", 30, signalYMap(1.5) + 4);
      ctx.fillText("0", 42, signalYMap(0) + 4);
      ctx.fillText("-1.5", 28, signalYMap(-1.5) + 4);
      ctx.fillText("+18", 32, derivativeYMap(18) + 4);
      ctx.fillText("0", 42, derivativeYMap(0) + 4);
      ctx.fillText("-18", 28, derivativeYMap(-18) + 4);
      ctx.save();
      ctx.translate(20, signalPlot.top + 66);
      ctx.rotate(-Math.PI / 2);
      ctx.fillText("input signal v(t)", 0, 0);
      ctx.restore();
      ctx.save();
      ctx.translate(20, derivativePlot.top + 88);
      ctx.rotate(-Math.PI / 2);
      ctx.fillText("derivative estimate", 0, 0);
      ctx.restore();

      ctx.fillStyle = "rgba(255,255,255,0.9)";
      ctx.strokeStyle = "#dbe7ea";
      drawRoundedRect(width - 344, 56, 302, 118, 8);
      ctx.font = "13px sans-serif";
      ctx.fillStyle = "#64748b";
      ctx.fillText("slate dashed: true signal", width - 326, 78);
      ctx.fillStyle = "#526047";
      ctx.fillText("green-gray: noisy input signal", width - 326, 100);
      ctx.fillStyle = "#7b5ea7";
      ctx.fillText("purple dashed: true derivative", width - 326, 122);
      ctx.fillStyle = "#d56b35";
      ctx.fillText("orange: raw finite difference", width - 326, 144);
      ctx.fillStyle = "#0e6d77";
      ctx.fillText("teal: dirty derivative", width - 326, 166);
    };
    const simulateAntiWindup = ({ enabled, method, limit, ki }) => {
      const dt = 0.005;
      const tEnd = 12;
      const steps = Math.round(tEnd / dt);
      const plantWn = 2.0;
      const plantZeta = 0.22;
      const plantB = 5.0;
      const reference = 1.0;
      const kp = 6.0;
      const kaw = 1.1;
      let y = 0;
      let yDot = 0;
      let xI = 0;
      const data = [];
      let saturationTime = 0;
      let maxY = -Infinity;
      let peakIntegrator = 0;
      let firstDesatTime = null;
      const clamp = (value, lo, hi) => Math.max(lo, Math.min(hi, value));

      for (let i = 0; i <= steps; i += 1) {
        const t = i * dt;
        const e = reference - y;
        const uUnsat = kp * e + ki * xI;
        const uSat = clamp(uUnsat, -limit, limit);
        const saturated = Math.abs(uUnsat - uSat) > 1e-8;
        data.push({ t, r: reference, y, e, uUnsat, uSat, xI, saturated });
        maxY = Math.max(maxY, y);
        peakIntegrator = Math.max(peakIntegrator, Math.abs(xI));
        if (saturated) saturationTime += dt;
        if (!saturated && t > 0.05 && firstDesatTime === null) firstDesatTime = t;

        let xIDot = e;
        if (enabled && method === "conditional" && saturated) {
          const drivesFurtherPositive = uUnsat > limit && e > 0;
          const drivesFurtherNegative = uUnsat < -limit && e < 0;
          if (drivesFurtherPositive || drivesFurtherNegative) xIDot = 0;
        }
        if (enabled && method === "backcalc") {
          xIDot = e + kaw * (uSat - uUnsat);
        }
        const yDDot = plantB * uSat - 2 * plantZeta * plantWn * yDot - plantWn * plantWn * y;
        y += dt * yDot;
        yDot += dt * yDDot;
        xI += dt * xIDot;
      }

      const final = data[data.length - 1] || { y: 0 };
      return {
        data,
        reference,
        metrics: {
          overshoot: Math.max(0, maxY - reference),
          saturationTime,
          firstDesatTime,
          finalError: reference - final.y,
          peakIntegrator,
        },
      };
    };
    const drawAntiWindupCanvas = (activity, active, baseline, limit) => {
      const canvas = activity.querySelector("[data-plot='anti-windup']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      const pad = { left: 72, right: 34, top: 42, bottom: 46 };
      const plotLeft = pad.left;
      const plotRight = width - pad.right;
      const finiteExtent = (values, fallbackMin, fallbackMax) => {
        const finiteValues = values.filter((value) => Number.isFinite(value));
        if (!finiteValues.length) return { yMin: fallbackMin, yMax: fallbackMax };
        const rawMin = Math.min(...finiteValues);
        const rawMax = Math.max(...finiteValues);
        const span = Math.max(1, rawMax - rawMin);
        const axisPad = 0.12 * span;
        return {
          yMin: Math.min(fallbackMin, rawMin - axisPad),
          yMax: Math.max(fallbackMax, rawMax + axisPad),
        };
      };
      const responseExtent = finiteExtent(
        baseline.data.concat(active.data).flatMap((point) => [point.y, point.r]),
        -0.15,
        1.55,
      );
      const controlExtent = finiteExtent(
        baseline.data.concat(active.data).flatMap((point) => [point.uUnsat, point.uSat, limit, -limit]),
        -1.8,
        4.8,
      );
      const integratorValues = baseline.data
        .concat(active.data)
        .map((point) => point.xI)
        .filter((value) => Number.isFinite(value));
      const minIntegrator = Math.min(0, ...integratorValues);
      const maxIntegrator = Math.max(0, ...integratorValues);
      const integratorSpan = Math.max(1, maxIntegrator - minIntegrator);
      const integratorPad = 0.12 * integratorSpan;
      const panels = [
        { top: 54, bottom: 205, label: "response", ...responseExtent },
        { top: 250, bottom: 390, label: "control", ...controlExtent },
        {
          top: 435,
          bottom: height - pad.bottom,
          label: "integrator",
          yMin: Math.min(-0.5, minIntegrator - integratorPad),
          yMax: Math.max(3.4, maxIntegrator + integratorPad),
        },
      ];
      const tEnd = Math.max(
        baseline.data[baseline.data.length - 1]?.t || 12,
        active.data[active.data.length - 1]?.t || 12,
      );
      const xMap = (t) => plotLeft + (t / tEnd) * (plotRight - plotLeft);
      const yMap = (panel, y) => {
        const clipped = Math.max(panel.yMin, Math.min(panel.yMax, y));
        return panel.bottom - ((clipped - panel.yMin) / (panel.yMax - panel.yMin)) * (panel.bottom - panel.top);
      };
      const drawPath = (series, panel, getter, color, lineWidth = 2.5, dash = []) => {
        ctx.save();
        ctx.beginPath();
        ctx.setLineDash(dash);
        series.forEach((point, index) => {
          const x = xMap(point.t);
          const y = yMap(panel, getter(point));
          if (index === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.strokeStyle = color;
        ctx.lineWidth = lineWidth;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.stroke();
        ctx.restore();
      };
      const drawPanel = (panel) => {
        ctx.strokeStyle = "#dce8eb";
        ctx.lineWidth = 1;
        for (let i = 0; i <= 6; i += 1) {
          const x = plotLeft + (i / 6) * (plotRight - plotLeft);
          ctx.beginPath();
          ctx.moveTo(x, panel.top);
          ctx.lineTo(x, panel.bottom);
          ctx.stroke();
        }
        for (let i = 0; i <= 4; i += 1) {
          const yValue = panel.yMin + (i / 4) * (panel.yMax - panel.yMin);
          const y = yMap(panel, yValue);
          ctx.beginPath();
          ctx.moveTo(plotLeft, y);
          ctx.lineTo(plotRight, y);
          ctx.stroke();
          ctx.fillStyle = "#526047";
          ctx.font = "12px sans-serif";
          ctx.fillText(fmt(yValue, 1), 26, y + 4);
        }
        ctx.strokeStyle = "#172d33";
        ctx.lineWidth = 1.8;
        ctx.strokeRect(plotLeft, panel.top, plotRight - plotLeft, panel.bottom - panel.top);
      };

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);
      ctx.font = "13px sans-serif";
      panels.forEach(drawPanel);

      const [responsePanel, controlPanel, integratorPanel] = panels;
      drawPath(baseline.data, responsePanel, (p) => p.y, "#9aa7ad", 2.3, [7, 5]);
      drawPath(active.data, responsePanel, (p) => p.y, "#0e6d77", 3);
      drawPath(active.data, responsePanel, (p) => p.r, "#8b5e24", 2, [5, 5]);

      drawPath(baseline.data, controlPanel, (p) => p.uUnsat, "#d56b35", 1.7, [7, 5]);
      drawPath(active.data, controlPanel, (p) => p.uUnsat, "#d56b35", 2.2);
      drawPath(active.data, controlPanel, (p) => p.uSat, "#0e6d77", 3);
      for (const bound of [limit, -limit]) {
        ctx.save();
        ctx.setLineDash([7, 5]);
        ctx.strokeStyle = "#8b5e24";
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        ctx.moveTo(plotLeft, yMap(controlPanel, bound));
        ctx.lineTo(plotRight, yMap(controlPanel, bound));
        ctx.stroke();
        ctx.restore();
      }

      drawPath(baseline.data, integratorPanel, (p) => p.xI, "#9aa7ad", 2.2, [7, 5]);
      drawPath(active.data, integratorPanel, (p) => p.xI, "#6f7f32", 3);

      ctx.fillStyle = "#244b54";
      ctx.font = "700 14px sans-serif";
      ctx.fillText("Step response y(t)", plotLeft, responsePanel.top - 18);
      ctx.fillText("Actuator command", plotLeft, controlPanel.top - 18);
      ctx.fillText("Integrator state xI", plotLeft, integratorPanel.top - 18);
      ctx.font = "13px sans-serif";
      ctx.fillStyle = "#526047";
      ctx.fillText("time (s)", plotRight - 56, height - 16);
      ctx.fillStyle = "rgba(255,255,255,0.92)";
      ctx.strokeStyle = "#dbe7ea";
      drawCanvasRoundedRect(ctx, width - 368, 54, 326, 136, 8);
      ctx.font = "13px sans-serif";
      ctx.fillStyle = "#9aa7ad";
      ctx.fillText("gray dashed: no anti-windup y, xI", width - 350, 78);
      ctx.fillStyle = "#d56b35";
      ctx.fillText("orange dashed: no-AW u_unsat", width - 350, 100);
      ctx.fillStyle = "#0e6d77";
      ctx.fillText("teal: selected controller / u_sat", width - 350, 122);
      ctx.fillStyle = "#d56b35";
      ctx.fillText("orange: selected u_unsat", width - 350, 144);
      ctx.fillStyle = "#6f7f32";
      ctx.fillText("green: selected integrator state", width - 350, 166);
    };
    const normalizeAdvancedActivityLayout = (activity) => {
      for (const element of activity.querySelectorAll("[class]")) {
        if (element.className.includes("advanced-grid–visual")) {
          element.classList.add("advanced-grid--visual");
        }
        if (element.className.includes("advanced-controls–compact")) {
          element.classList.add("advanced-controls--compact");
        }
      }

      const buildLooseReadout = (parent = activity) => {
        const pairs = [];
        for (const child of Array.from(parent.children)) {
          const value = child.nextElementSibling;
          if (child.tagName === "STRONG" && value?.matches?.("span[data-output]")) {
            pairs.push([child, value]);
          }
        }
        if (!pairs.length) return null;

        const readout = document.createElement("div");
        readout.className = "advanced-readout";
        for (const [label, value] of pairs) {
          const metric = document.createElement("div");
          metric.className = "advanced-metric";
          metric.append(label, value);
          readout.append(metric);
        }
        return readout;
      };

      let grid = activity.querySelector(":scope > .advanced-grid");
      const labelParent = grid || activity;
      const labels = Array.from(labelParent.children).filter((child) =>
        child.tagName === "LABEL" && child.querySelector("[data-input]")
      );
      const readout = activity.querySelector(":scope > .advanced-readout")
        || grid?.querySelector(":scope > .advanced-readout")
        || buildLooseReadout(activity)
        || (grid ? buildLooseReadout(grid) : null);
      if (!labels.length && !readout) return;

      if (!grid) {
        grid = document.createElement("div");
        grid.className = "advanced-grid";
        const anchor = labels[0] || readout;
        anchor.before(grid);
      }

      if (labels.length) {
        const controls = grid.querySelector(":scope > .advanced-controls") || document.createElement("div");
        controls.className = "advanced-controls";
        labels.forEach((label) => controls.append(label));
        if (!controls.parentElement) {
          const firstVisual = grid.querySelector(":scope > .advanced-visual");
          if (firstVisual) firstVisual.before(controls);
          else grid.prepend(controls);
        }
      }
      if (readout && !grid.contains(readout)) grid.append(readout);
    };
    const drawLinearizationCanvas = (activity, x0, u0, x, u) => {
      const canvas = activity.querySelector("[data-plot='linearization']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);

      const f = (px, pu) => px * px + pu;
      const plane = (px, pu) => x0 * x0 + u0 + 2 * x0 * (px - x0) + (pu - u0);
      const project = (px, pu, pf) => ({
        x: width * 0.48 + px * 38 + pu * 12,
        y: height * 0.62 - pu * 8 - pf * 6,
      });
      const drawLine = (points, color, lineWidth = 1.5) => {
        ctx.beginPath();
        points.forEach((point, index) => {
          if (index === 0) ctx.moveTo(point.x, point.y);
          else ctx.lineTo(point.x, point.y);
        });
        ctx.strokeStyle = color;
        ctx.lineWidth = lineWidth;
        ctx.stroke();
      };
      const fillQuad = (points, fill, stroke) => {
        ctx.beginPath();
        points.forEach((point, index) => {
          if (index === 0) ctx.moveTo(point.x, point.y);
          else ctx.lineTo(point.x, point.y);
        });
        ctx.closePath();
        ctx.fillStyle = fill;
        ctx.fill();
        ctx.strokeStyle = stroke;
        ctx.lineWidth = 0.8;
        ctx.stroke();
      };

      const xValues = [];
      for (let gx = -3; gx <= 3.001; gx += 0.5) xValues.push(Number(gx.toFixed(2)));
      const uValues = [];
      for (let gu = -9; gu <= 3.001; gu += 1) uValues.push(Number(gu.toFixed(2)));

      const surfaceQuads = [];
      for (let ix = 0; ix < xValues.length - 1; ix += 1) {
        for (let iu = 0; iu < uValues.length - 1; iu += 1) {
          const xA = xValues[ix];
          const xB = xValues[ix + 1];
          const uA = uValues[iu];
          const uB = uValues[iu + 1];
          surfaceQuads.push({
            depth: xA + xB + uA + uB,
            points: [
              project(xA, uA, f(xA, uA)),
              project(xB, uA, f(xB, uA)),
              project(xB, uB, f(xB, uB)),
              project(xA, uB, f(xA, uB)),
            ],
          });
        }
      }
      surfaceQuads
        .sort((a, b) => a.depth - b.depth)
        .forEach((quad) => fillQuad(quad.points, "rgba(14, 109, 119, 0.16)", "rgba(14, 109, 119, 0.42)"));

      const planeRangeX = [Math.max(-3, x0 - 1.8), Math.min(3, x0 + 1.8)];
      const planeRangeU = [Math.max(-9, u0 - 3.0), Math.min(3, u0 + 3.0)];
      const planeXs = [];
      for (let gx = planeRangeX[0]; gx <= planeRangeX[1] + 0.001; gx += 0.45) planeXs.push(Number(gx.toFixed(2)));
      const planeUs = [];
      for (let gu = planeRangeU[0]; gu <= planeRangeU[1] + 0.001; gu += 0.75) planeUs.push(Number(gu.toFixed(2)));
      const planeQuads = [];
      for (let ix = 0; ix < planeXs.length - 1; ix += 1) {
        for (let iu = 0; iu < planeUs.length - 1; iu += 1) {
          const xA = planeXs[ix];
          const xB = planeXs[ix + 1];
          const uA = planeUs[iu];
          const uB = planeUs[iu + 1];
          planeQuads.push({
            depth: xA + xB + uA + uB + 0.01,
            points: [
              project(xA, uA, plane(xA, uA)),
              project(xB, uA, plane(xB, uA)),
              project(xB, uB, plane(xB, uB)),
              project(xA, uB, plane(xA, uB)),
            ],
          });
        }
      }
      planeQuads
        .sort((a, b) => a.depth - b.depth)
        .forEach((quad) => fillQuad(quad.points, "rgba(213, 107, 53, 0.24)", "rgba(213, 107, 53, 0.62)"));

      const origin = { x: width * 0.18, y: height * 0.82 };
      const xAxis = { x: width * 0.38, y: height * 0.82 };
      const uAxis = { x: width * 0.31, y: height * 0.74 };
      const fAxis = { x: width * 0.18, y: height * 0.56 };
      drawLine([origin, xAxis], "#172d33", 1.5);
      drawLine([origin, uAxis], "#172d33", 1.5);
      drawLine([origin, fAxis], "#172d33", 1.5);
      ctx.fillStyle = "#172d33";
      ctx.font = "13px sans-serif";
      ctx.fillText("x", xAxis.x + 6, xAxis.y + 4);
      ctx.fillText("u", uAxis.x + 6, uAxis.y + 4);
      ctx.fillText("f(x,u)", fAxis.x + 6, fAxis.y - 2);

      const point = project(x0, u0, f(x0, u0));
      ctx.fillStyle = "#d56b35";
      ctx.beginPath();
      ctx.arc(point.x, point.y, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = "#172d33";
      ctx.fillText("operating point", point.x + 8, point.y - 8);
      const nearby = project(x, u, f(x, u));
      ctx.fillStyle = "#0e6d77";
      ctx.beginPath();
      ctx.arc(nearby.x, nearby.y, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = "#172d33";
      ctx.fillText("nearby point", nearby.x + 8, nearby.y + 16);

      ctx.fillStyle = "#0e6d77";
      ctx.fillRect(16, 16, 14, 8);
      ctx.fillStyle = "#172d33";
      ctx.fillText("nonlinear surface", 36, 24);
      ctx.fillStyle = "#d56b35";
      ctx.fillRect(16, 40, 14, 8);
      ctx.fillStyle = "#172d33";
      ctx.fillText("tangent plane", 36, 48);
    };
    const drawLinearizationErrorCanvas = (activity, x0, u0, x, u) => {
      const canvas = activity.querySelector("[data-plot='linearization-error']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);

      const f = (px, pu) => px * px + pu;
      const plane = (px, pu) => x0 * x0 + u0 + 2 * x0 * (px - x0) + (pu - u0);
      const error = (px, pu) => f(px, pu) - plane(px, pu);
      const plot = {
        left: 58,
        right: width - 34,
        top: 28,
        bottom: height - 46,
      };
      const xMin = -3;
      const xMax = 3;
      const uMin = -9;
      const uMax = 3;
      const sx = (px) => plot.left + ((px - xMin) / (xMax - xMin)) * (plot.right - plot.left);
      const sy = (pu) => plot.bottom - ((pu - uMin) / (uMax - uMin)) * (plot.bottom - plot.top);
      const colorFor = (value) => {
        const t = Math.max(0, Math.min(1, Math.abs(value) / 6));
        const light = 96 - t * 42;
        const alpha = 0.34 + t * 0.56;
        return `hsla(190, 64%, ${light}%, ${alpha})`;
      };

      const cols = 48;
      const rows = 36;
      for (let ix = 0; ix < cols; ix += 1) {
        for (let iu = 0; iu < rows; iu += 1) {
          const xA = xMin + (ix / cols) * (xMax - xMin);
          const xB = xMin + ((ix + 1) / cols) * (xMax - xMin);
          const uA = uMin + (iu / rows) * (uMax - uMin);
          const uB = uMin + ((iu + 1) / rows) * (uMax - uMin);
          const e = error((xA + xB) / 2, (uA + uB) / 2);
          ctx.fillStyle = colorFor(e);
          ctx.fillRect(sx(xA), sy(uB), sx(xB) - sx(xA) + 1, sy(uA) - sy(uB) + 1);
        }
      }

      const drawMarker = (px, pu, color, label, dy) => {
        const mx = sx(px);
        const my = sy(pu);
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(mx, my, 5, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "#172d33";
        ctx.font = "13px sans-serif";
        ctx.fillText(label, mx + 8, my + dy);
      };
      ctx.strokeStyle = "rgba(213, 107, 53, 0.85)";
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(sx(x0), plot.top);
      ctx.lineTo(sx(x0), plot.bottom);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.strokeStyle = "#172d33";
      ctx.lineWidth = 1.4;
      ctx.strokeRect(plot.left, plot.top, plot.right - plot.left, plot.bottom - plot.top);
      ctx.fillStyle = "#172d33";
      ctx.font = "13px sans-serif";
      ctx.fillText("x", (plot.left + plot.right) / 2, height - 14);
      ctx.save();
      ctx.translate(18, (plot.top + plot.bottom) / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.fillText("u", 0, 0);
      ctx.restore();

      [-3, -1.5, 0, 1.5, 3].forEach((tick) => {
        const tx = sx(tick);
        ctx.beginPath();
        ctx.moveTo(tx, plot.bottom);
        ctx.lineTo(tx, plot.bottom + 5);
        ctx.stroke();
        ctx.fillText(String(tick), tx - 9, plot.bottom + 20);
      });
      [-9, -6, -3, 0, 3].forEach((tick) => {
        const ty = sy(tick);
        ctx.beginPath();
        ctx.moveTo(plot.left - 5, ty);
        ctx.lineTo(plot.left, ty);
        ctx.stroke();
        ctx.fillText(String(tick), 30, ty + 4);
      });

      drawMarker(x0, u0, "#d56b35", "operating point", -8);
      drawMarker(x, u, "#0e6d77", "nearby point", 17);
      ctx.fillStyle = "#172d33";
      ctx.font = "13px sans-serif";
      ctx.fillText(`nearby error = ${fmt(error(x, u), 3)}`, plot.left, 18);
      ctx.fillText("dashed line: x = x0", plot.right - 134, 18);
    };
    const ensureAutoVisual = (activity, title, caption) => {
      if (activity.querySelector(".advanced-visual")) return null;
      const figure = document.createElement("figure");
      figure.className = "advanced-visual advanced-visual--auto";
      figure.innerHTML = `
        <svg viewBox="0 0 420 210" role="img" aria-label="${title}">
          <g data-auto-visual></g>
        </svg>
        <figcaption>${caption}</figcaption>
      `;
      const grid = activity.querySelector(".advanced-grid");
      if (grid) grid.insertAdjacentElement("afterend", figure);
      else activity.append(figure);
      return figure.querySelector("[data-auto-visual]");
    };
    const autoVisual = (activity, title, caption) =>
      activity.querySelector("[data-auto-visual]") || ensureAutoVisual(activity, title, caption);
    const drawAutoVisual = (activity, title, caption, content, viewBox = "0 0 420 210") => {
      const target = autoVisual(activity, title, caption);
      if (target) {
        target.closest("svg")?.setAttribute("viewBox", viewBox);
        target.innerHTML = content;
      }
    };
    const block = (x, y, w, h, label, fill = "#ffffff", stroke = "#172d33") => `
      <rect x="${x}" y="${y}" width="${w}" height="${h}" rx="10" fill="${fill}" stroke="${stroke}" stroke-width="3"></rect>
      <text x="${x + w / 2}" y="${y + h / 2 + 5}" text-anchor="middle" font-size="15" fill="#172d33">${label}</text>
    `;
    const arrow = (x1, y1, x2, y2, color = "#172d33") => `
      <line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${color}" stroke-width="3"></line>
      <path d="M${x2} ${y2} l-10 -6 v12 z" fill="${color}" transform="rotate(${Math.atan2(y2 - y1, x2 - x1) * 180 / Math.PI} ${x2} ${y2})"></path>
    `;
    const drawInnovationCanvas = (activity, l1, l2) => {
      const canvas = activity.querySelector("[data-plot='innovation']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);

      const dt = 0.02;
      const tEnd = 12;
      let x1 = 1;
      let x2 = 0.4;
      let h1 = -1;
      let h2 = -0.2;
      const rows = [];
      let maxAbs = 1;
      for (let k = 0; k <= tEnd / dt; k += 1) {
        const t = k * dt;
        const innovation = x1 - h1;
        rows.push({ t, x1, x2, h1, h2, innovation });
        maxAbs = Math.max(maxAbs, Math.abs(x1), Math.abs(h1), Math.abs(x2), Math.abs(h2));
        const x1dot = x2;
        const x2dot = 0;
        const h1dot = h2 + l1 * innovation;
        const h2dot = l2 * innovation;
        x1 += dt * x1dot;
        x2 += dt * x2dot;
        h1 += dt * h1dot;
        h2 += dt * h2dot;
        if (Math.abs(h1) > 1e4 || Math.abs(h2) > 1e4) break;
      }

      const margin = { left: 58, right: 22, top: 34, bottom: 44 };
      const plotW = width - margin.left - margin.right;
      const plotH = height - margin.top - margin.bottom;
      const sx = (t) => margin.left + (t / tEnd) * plotW;
      const sy = (v) => margin.top + plotH * (0.5 - v / (2.2 * maxAbs));
      const drawTrace = (key, color, dash = []) => {
        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.2;
        ctx.setLineDash(dash);
        ctx.beginPath();
        rows.forEach((row, i) => {
          const x = sx(row.t);
          const y = sy(row[key]);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.restore();
      };

      ctx.strokeStyle = "#dbe7ea";
      ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i += 1) {
        const y = margin.top + (plotH * i) / 4;
        ctx.beginPath();
        ctx.moveTo(margin.left, y);
        ctx.lineTo(width - margin.right, y);
        ctx.stroke();
      }
      ctx.strokeStyle = "#172d33";
      ctx.strokeRect(margin.left, margin.top, plotW, plotH);
      drawTrace("x1", "#0e6d77");
      drawTrace("h1", "#0e6d77", [7, 5]);
      drawTrace("x2", "#d56b35");
      drawTrace("h2", "#d56b35", [7, 5]);

      ctx.fillStyle = "#172d33";
      ctx.font = "13px sans-serif";
      ctx.fillText("time (s)", width - 82, height - 16);
      ctx.save();
      ctx.translate(18, height / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.fillText("state and estimate", 0, 0);
      ctx.restore();
      ctx.fillStyle = "rgba(255,255,255,0.9)";
      ctx.strokeStyle = "#dbe7ea";
      drawCanvasRoundedRect(ctx, width - 260, 42, 220, 96, 8);
      ctx.fillStyle = "#0e6d77";
      ctx.fillText("solid teal: position x1", width - 242, 66);
      ctx.fillText("dashed teal: estimate xhat1", width - 242, 88);
      ctx.fillStyle = "#d56b35";
      ctx.fillText("solid orange: velocity x2", width - 242, 110);
      ctx.fillText("dashed orange: estimate xhat2", width - 242, 132);
    };

    const drawCarObservabilityVisual = (activity, c1, c2) => {
      const svg = activity.querySelector("[data-car-observability-visual]");
      if (!svg) return;
      const observable = Math.abs(c1) > 1e-9;
      const rank = observable ? 2 : (Math.abs(c2) > 1e-9 ? 1 : 0);
      const sensorX = 418 + Math.max(-90, Math.min(90, c1 * 45));
      const velocityArrow = c2 === 0 ? "" : `
        <line x1="478" y1="122" x2="${478 + Math.sign(c2) * 72}" y2="122" stroke="#d56b35" stroke-width="5"></line>
        <path d="M${478 + Math.sign(c2) * 82} 122 l-14 -9 v18 z" fill="#d56b35" transform="${c2 < 0 ? `rotate(180 ${478 + Math.sign(c2) * 82} 122)` : ""}"></path>
      `;
      svg.innerHTML = `
        <rect x="0" y="0" width="760" height="300" rx="16" fill="#fbfdfe"></rect>
        <line x1="70" y1="210" x2="690" y2="210" stroke="#9fb7bd" stroke-width="4"></line>
        <line x1="120" y1="210" x2="120" y2="198" stroke="#9fb7bd" stroke-width="3"></line>
        <line x1="600" y1="210" x2="600" y2="198" stroke="#9fb7bd" stroke-width="3"></line>
        <text x="112" y="236" font-size="14" fill="#526047">position</text>
        <g transform="translate(390 142)">
          <rect x="-58" y="-34" width="116" height="48" rx="12" fill="#eaf4f5" stroke="#172d33" stroke-width="3"></rect>
          <circle cx="-34" cy="22" r="14" fill="#172d33"></circle>
          <circle cx="34" cy="22" r="14" fill="#172d33"></circle>
          <text x="0" y="-48" text-anchor="middle" font-size="15" fill="#172d33">x1 = position, x2 = velocity</text>
        </g>
        ${velocityArrow}
        <g transform="translate(${sensorX} 68)">
          <path d="M0 0 l24 54 h-48 z" fill="${observable ? "#0e6d77" : "#d56b35"}" opacity="0.9"></path>
          <circle cx="0" cy="64" r="10" fill="#172d33"></circle>
          <text x="0" y="-12" text-anchor="middle" font-size="14" fill="#172d33">sensor y = ${fmt(c1)}x1 + ${fmt(c2)}x2</text>
        </g>
        <rect x="84" y="28" width="248" height="86" rx="10" fill="#ffffff" stroke="#dbe7ea"></rect>
        <text x="104" y="58" font-size="15" fill="#244b54">O = [ C ; CA ]</text>
        <text x="104" y="84" font-size="15" fill="#172d33">= [[${fmt(c1)}, ${fmt(c2)}], [0, ${fmt(c1)}]]</text>
        <text x="104" y="106" font-size="14" fill="${observable ? "#0e6d77" : "#9c4735"}">rank = ${rank}: ${observable ? "observable" : "not fully observable"}</text>
        <text x="438" y="270" text-anchor="middle" font-size="15" fill="#526047">${observable ? "Position information lets velocity be inferred from change over time." : "Without position in y, absolute position remains hidden."}</text>
      `;
    };
    const ch14DisturbanceParams = {
      a: [[0, 1], [2, 3]],
      b: [0, 4],
      c: [5, 0],
      k: [0.75, 1.1035],
      kr: 0.05,
      observerL: [3.428, 30.684],
      observerPairA1: 14.14,
      observerPairA0: 100,
    };
    const ch14AugmentedObserverGain = (poleMagnitude) => {
      const p = Math.max(0.1, poleMagnitude);
      const { observerPairA1, observerPairA0 } = ch14DisturbanceParams;
      const l1 = (observerPairA1 + p + 3) / 5;
      const l2 = (observerPairA0 + observerPairA1 * p + 15 * l1 + 2) / 5;
      const ld = (observerPairA0 * p) / 20;
      return [l1, l2, ld];
    };
    const simulateCh14DisturbanceObserver = ({ disturbance, poleMagnitude, reference, mode = "augmented" }) => {
      const { a, b, c, k, kr, observerL } = ch14DisturbanceParams;
      const l2 = ch14AugmentedObserverGain(poleMagnitude);
      const dt = 0.01;
      const tEnd = 14;
      const disturbanceTime = 1;
      const rows = [];
      let x = [0, 0];
      let xhat = [0, 0];
      let xobs = [0, 0, 0];
      let u = kr * reference;
      for (let step = 0; step <= tEnd / dt; step += 1) {
        const t = step * dt;
        const dApplied = t >= disturbanceTime ? disturbance : 0;
        const y = c[0] * x[0] + c[1] * x[1];
        const dhat = mode === "augmented" ? xobs[2] : 0;
        rows.push({
          t,
          y,
          r: reference,
          dhat,
          trueD: dApplied,
          innovation: mode === "augmented"
            ? y - (c[0] * xobs[0] + c[1] * xobs[1])
            : y - (c[0] * xhat[0] + c[1] * xhat[1]),
          u,
          error: reference - y,
        });

        if (mode === "augmented") {
          u = kr * reference - k[0] * xobs[0] - k[1] * xobs[1] - xobs[2];
          const innovation = y - (c[0] * xobs[0] + c[1] * xobs[1]);
          const xobsDot = [
            xobs[1] + l2[0] * innovation,
            a[1][0] * xobs[0] + a[1][1] * xobs[1] + b[1] * xobs[2] + b[1] * u + l2[1] * innovation,
            l2[2] * innovation,
          ];
          xobs = xobs.map((value, index) => value + dt * xobsDot[index]);
        } else {
          u = kr * reference - k[0] * xhat[0] - k[1] * xhat[1];
          const innovation = y - (c[0] * xhat[0] + c[1] * xhat[1]);
          const xhatDot = [
            xhat[1] + observerL[0] * innovation,
            a[1][0] * xhat[0] + a[1][1] * xhat[1] + b[1] * u + observerL[1] * innovation,
          ];
          xhat = xhat.map((value, index) => value + dt * xhatDot[index]);
        }
        const xDot = [
          x[1],
          a[1][0] * x[0] + a[1][1] * x[1] + b[1] * (u + dApplied),
        ];
        x = x.map((value, index) => value + dt * xDot[index]);
      }

      const last = rows[rows.length - 1] || {};
      const threshold = Math.max(0.01, 0.05 * Math.abs(disturbance));
      let settlingTime = Number.NaN;
      if (mode === "augmented" && Math.abs(disturbance) > 1e-9) {
        for (let i = 0; i < rows.length; i += 1) {
          if (rows[i].t < disturbanceTime) continue;
          const remainsSettled = rows.slice(i).every((row) => Math.abs(row.dhat - disturbance) <= threshold);
          if (remainsSettled) {
            settlingTime = rows[i].t - disturbanceTime;
            break;
          }
        }
      }
      return {
        rows,
        metrics: {
          finalY: last.y ?? 0,
          finalError: last.error ?? 0,
          finalDhat: last.dhat ?? 0,
          finalInnovation: last.innovation ?? 0,
          settlingTime,
        },
      };
    };
    const drawCh14ObserverCanvas = (activity, plotKey, traces, options = {}) => {
      const canvas = activity.querySelector(`[data-plot='${plotKey}']`);
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);

      const panels = options.panels || [
        { title: "Output y(t)", value: (row) => row.y, guides: [{ value: (row) => row.r, label: "reference r", color: "#5c6bc0", dash: [6, 5] }] },
        { title: "Disturbance estimate", value: (row) => row.dhat, guides: [{ value: (row) => row.trueD, label: "d", color: "#8b5e24", dash: [6, 5] }] },
        { title: "Innovation y - C x_hat", value: (row) => row.innovation },
        { title: "Control input u(t)", value: (row) => row.u },
      ];
      const margin = { left: 66, right: 26, top: 34, bottom: 36 };
      const gap = 30;
      const panelH = (height - margin.top - margin.bottom - gap * (panels.length - 1)) / panels.length;
      const plotLeft = margin.left;
      const plotRight = width - margin.right;
      const tMax = Math.max(...traces.flatMap((trace) => trace.rows.map((row) => row.t)), 8);
      const colors = ["#0e6d77", "#8b5e24", "#d56b35", "#69751d"];

      const sx = (t) => plotLeft + (t / tMax) * (plotRight - plotLeft);
      const valueAt = (value, row) => typeof value === "function" ? value(row) : row[value];
      const drawSeries = (rows, panel, value, color, dash = [], widthPx = 2.6) => {
        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = widthPx;
        ctx.setLineDash(dash);
        ctx.beginPath();
        rows.forEach((row, index) => {
          const x = sx(row.t);
          const y = panel.sy(valueAt(value, row));
          if (index === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.restore();
      };

      panels.forEach((panelDef, index) => {
        const top = margin.top + index * (panelH + gap);
        const bottom = top + panelH;
        const allValues = traces.flatMap((trace) => trace.rows.flatMap((row) => {
          const values = [valueAt(panelDef.value, row)];
          for (const guide of panelDef.guides || []) values.push(valueAt(guide.value, row));
          return values;
        })).filter(Number.isFinite);
        const maxAbs = Math.max(0.5, ...allValues.map((value) => Math.abs(value))) * 1.12;
        const sy = (value) => bottom - ((Math.max(-maxAbs, Math.min(maxAbs, value)) + maxAbs) / (2 * maxAbs)) * panelH;
        const panel = { top, bottom, sy };

        ctx.strokeStyle = "#dbe7ea";
        ctx.lineWidth = 1;
        for (let grid = 0; grid <= 4; grid += 1) {
          const y = top + (panelH * grid) / 4;
          ctx.beginPath();
          ctx.moveTo(plotLeft, y);
          ctx.lineTo(plotRight, y);
          ctx.stroke();
        }
        ctx.strokeStyle = "#172d33";
        ctx.lineWidth = 1.7;
        ctx.strokeRect(plotLeft, top, plotRight - plotLeft, panelH);
        ctx.strokeStyle = "#9fb3b9";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(plotLeft, sy(0));
        ctx.lineTo(plotRight, sy(0));
        ctx.stroke();

        for (const guide of panelDef.guides || []) {
          const guideRows = traces[0]?.rows || [];
          drawSeries(guideRows, panel, guide.value, guide.color || "#8b5e24", guide.dash || [6, 5], 2);
        }
        traces.forEach((trace, traceIndex) => {
          drawSeries(trace.rows, panel, panelDef.value, trace.color || colors[traceIndex % colors.length], trace.dash || [], trace.width || 2.6);
        });

        ctx.fillStyle = "#244b54";
        ctx.font = "700 14px sans-serif";
        ctx.fillText(panelDef.title, plotLeft, top - 10);
        ctx.fillStyle = "#526047";
        ctx.font = "12px sans-serif";
        ctx.fillText(`+${fmt(maxAbs, 2)}`, 14, top + 12);
        ctx.fillText("0", 36, sy(0) + 4);
        ctx.fillText(`-${fmt(maxAbs, 2)}`, 14, bottom);
      });

      ctx.fillStyle = "#172d33";
      ctx.font = "13px sans-serif";
      ctx.fillText("time (s)", plotRight - 56, height - 12);
      ctx.fillStyle = "rgba(255,255,255,0.92)";
      ctx.strokeStyle = "#dbe7ea";
      const legendWidth = options.legendWidth || 300;
      const legendHeight = 26 + 22 * traces.length + (options.extraLegend ? 22 : 0);
      drawCanvasRoundedRect(ctx, width - legendWidth - 28, 18, legendWidth, legendHeight, 8);
      traces.forEach((trace, index) => {
        const y = 42 + index * 22;
        ctx.save();
        ctx.strokeStyle = trace.color || colors[index % colors.length];
        ctx.lineWidth = trace.width || 2.6;
        ctx.setLineDash(trace.dash || []);
        ctx.beginPath();
        ctx.moveTo(width - legendWidth - 10, y - 5);
        ctx.lineTo(width - legendWidth + 22, y - 5);
        ctx.stroke();
        ctx.restore();
        ctx.fillStyle = "#172d33";
        ctx.font = "13px sans-serif";
        ctx.fillText(trace.label, width - legendWidth + 32, y);
      });
      if (options.extraLegend) {
        ctx.fillStyle = "#526047";
        ctx.fillText(options.extraLegend, width - legendWidth + 32, 42 + traces.length * 22);
      }
    };
    const rankMatrix = (matrix) => {
      const a = matrix.map((row) => row.slice());
      const rows = a.length;
      const cols = a[0]?.length || 0;
      let rank = 0;
      for (let col = 0; col < cols; col += 1) {
        let pivot = rank;
        for (let row = rank + 1; row < rows; row += 1) {
          if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
        }
        if (Math.abs(a[pivot][col]) < 1e-9) continue;
        [a[rank], a[pivot]] = [a[pivot], a[rank]];
        const pivotValue = a[rank][col];
        for (let c = col; c < cols; c += 1) a[rank][c] /= pivotValue;
        for (let row = 0; row < rows; row += 1) {
          if (row === rank) continue;
          const scale = a[row][col];
          for (let c = col; c < cols; c += 1) a[row][c] -= scale * a[rank][c];
        }
        rank += 1;
      }
      return rank;
    };
    const multiplyMatrix = (left, right) => left.map((row) =>
      right[0].map((_, colIndex) =>
        row.reduce((sum, value, innerIndex) => sum + value * right[innerIndex][colIndex], 0)
      )
    );
    const ch14DisturbanceObservability = (outputKey, channelKey) => {
      const cChoices = {
        position: [5, 0],
        velocity: [0, 1],
        sum: [1, 1],
        none: [0, 0],
      };
      const bdChoices = {
        input: [0, 4],
        position: [1, 0],
        velocity: [0, 1],
        none: [0, 0],
      };
      const c = cChoices[outputKey] || cChoices.position;
      const bd = bdChoices[channelKey] || bdChoices.input;
      const a2 = [
        [0, 1, bd[0]],
        [2, 3, bd[1]],
        [0, 0, 0],
      ];
      const c2 = [[c[0], c[1], 0]];
      const ca = multiplyMatrix(c2, a2);
      const ca2 = multiplyMatrix(ca, a2);
      const o = [c2[0], ca[0], ca2[0]];
      return { c, bd, o, rank: rankMatrix(o) };
    };
    const drawCh14DisturbanceObservabilityVisual = (activity, result) => {
      const svg = activity.querySelector("[data-disturbance-observability-visual]");
      if (!svg) return;
      const observable = result.rank === 3;
      const distStroke = result.bd[0] === 0 && result.bd[1] === 0 ? "#9aa7ad" : "#d56b35";
      const sensorStroke = result.c[0] === 0 && result.c[1] === 0 ? "#9c4735" : "#0e6d77";
      const distSummary = result.bd[0] === 0 && result.bd[1] === 0
        ? ["Disturbance: no plant effect."]
        : [`Disturbance path: Bd = [${fmt(result.bd[0])}, ${fmt(result.bd[1])}]^T.`, "It must affect states the sensor can reveal."];
      const sensorSummary = result.c[0] === 0 && result.c[1] === 0
        ? ["Sensor: no useful plant measurement."]
        : [`Sensor: y = [${fmt(result.c[0])}, ${fmt(result.c[1])}] x.`];
      const rankSummary = observable
        ? ["Output history can estimate", "x1, x2, and d."]
        : ["At least one augmented state is", "hidden from this measurement."];
      const svgLines = (lines, x, y, fill, size = 15, weight = 700, lineHeight = 20) =>
        lines.map((line, index) =>
          `<text x="${x}" y="${y + index * lineHeight}" font-size="${size}" font-weight="${weight}" fill="${fill}">${line}</text>`
        ).join("");
      svg.innerHTML = `
        <rect x="0" y="0" width="960" height="430" rx="18" fill="#fbfdfe"></rect>
        <defs>
          <marker id="ch14-dist-arrow" markerWidth="12" markerHeight="12" refX="10" refY="6" orient="auto">
            <path d="M0 0 L12 6 L0 12 Z" fill="#172d33"></path>
          </marker>
          <marker id="ch14-dist-orange-arrow" markerWidth="12" markerHeight="12" refX="10" refY="6" orient="auto">
            <path d="M0 0 L12 6 L0 12 Z" fill="${distStroke}"></path>
          </marker>
        </defs>
        <text x="42" y="44" font-size="22" font-weight="800" fill="#172d33">Can the disturbance leave a measurable signature?</text>
        <text x="42" y="73" font-size="15" fill="#526047">Trace whether d changes the plant states and whether the selected sensor can see that change.</text>

        <line x1="72" y1="185" x2="285" y2="185" stroke="#172d33" stroke-width="4" marker-end="url(#ch14-dist-arrow)"></line>
        <text x="100" y="166" font-size="17" font-weight="700" fill="#172d33">known input u</text>

        <rect x="300" y="132" width="220" height="106" rx="15" fill="#ffffff" stroke="#172d33" stroke-width="3.5"></rect>
        <text x="410" y="169" text-anchor="middle" font-size="22" font-weight="800" fill="#172d33">Plant dynamics</text>
        <text x="410" y="198" text-anchor="middle" font-size="16" fill="#526047">state x = [x1, x2]</text>
        <text x="410" y="222" text-anchor="middle" font-size="15" fill="#526047">A maps current state to future state</text>

        <line x1="410" y1="88" x2="410" y2="121" stroke="${distStroke}" stroke-width="5" marker-end="url(#ch14-dist-orange-arrow)"></line>
        <text x="430" y="102" font-size="16" font-weight="800" fill="${distStroke}">disturbance d</text>
        <text x="430" y="121" font-size="13" fill="${distStroke}">constant, unknown</text>

        <line x1="535" y1="185" x2="704" y2="185" stroke="#172d33" stroke-width="4" marker-end="url(#ch14-dist-arrow)"></line>
        <text x="574" y="166" font-size="17" font-weight="700" fill="#172d33">state history</text>

        <g transform="translate(730 128)">
          <rect x="0" y="0" width="154" height="114" rx="14" fill="#ffffff" stroke="${sensorStroke}" stroke-width="3.5"></rect>
          <path d="M28 72 L58 38 L92 62 L126 30" fill="none" stroke="${sensorStroke}" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"></path>
          <circle cx="28" cy="72" r="5" fill="${sensorStroke}"></circle>
          <circle cx="58" cy="38" r="5" fill="${sensorStroke}"></circle>
          <circle cx="92" cy="62" r="5" fill="${sensorStroke}"></circle>
          <circle cx="126" cy="30" r="5" fill="${sensorStroke}"></circle>
          <text x="77" y="100" text-anchor="middle" font-size="18" font-weight="800" fill="#172d33">Sensor C</text>
        </g>
        <line x1="884" y1="185" x2="925" y2="185" stroke="#172d33" stroke-width="4" marker-end="url(#ch14-dist-arrow)"></line>
        <text x="900" y="166" font-size="17" font-weight="700" fill="#172d33">y</text>

        <rect x="42" y="286" width="410" height="102" rx="14" fill="#ffffff" stroke="#dbe7ea" stroke-width="2"></rect>
        ${svgLines(distSummary, 64, 314, distStroke, 15, 800, 19)}
        ${svgLines(sensorSummary, 64, distSummary.length > 1 ? 355 : 342, sensorStroke, 15, 800, 19)}
        <text x="64" y="376" font-size="14" fill="#526047">Selections change the observability matrix rows.</text>

        <rect x="500" y="286" width="418" height="102" rx="14" fill="${observable ? "#edf7f8" : "#fff3ee"}" stroke="${observable ? "#0e6d77" : "#9c4735"}" stroke-width="2.5"></rect>
        <text x="524" y="319" font-size="18" font-weight="900" fill="${observable ? "#0e6d77" : "#9c4735"}">Rank = ${result.rank} / 3: ${observable ? "observable" : "not fully observable"}</text>
        ${svgLines(rankSummary, 524, 347, "#244b54", 15, 600, 18)}
        <text x="524" y="383" font-size="14" fill="#526047">Full rank is needed before pole placement.</text>
      `;
    };
    const logspace = (minExp, maxExp, count) =>
      Array.from({ length: count }, (_, i) => 10 ** (minExp + ((maxExp - minExp) * i) / (count - 1)));
    const unwrapDegrees = (degrees) => {
      const out = [];
      let offset = 0;
      let previous = degrees[0] ?? 0;
      for (const value of degrees) {
        let current = value + offset;
        while (current - previous > 180) {
          offset -= 360;
          current = value + offset;
        }
        while (current - previous < -180) {
          offset += 360;
          current = value + offset;
        }
        out.push(current);
        previous = current;
      }
      return out;
    };
    const drawTwoPanelPlot = (canvas, series, options) => {
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      const plot = { left: 64, right: width - 24, top1: 32, bottom1: Math.floor(height * 0.48), top2: Math.floor(height * 0.58), bottom2: height - 42 };
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);
      const logX = options.logX !== false;
      const xMin = options.xMin;
      const xMax = options.xMax;
      const sx = (x) => {
        if (!logX) return plot.left + ((x - xMin) / (xMax - xMin)) * (plot.right - plot.left);
        return plot.left + ((Math.log10(x) - Math.log10(xMin)) / (Math.log10(xMax) - Math.log10(xMin))) * (plot.right - plot.left);
      };
      const sy1 = (y) => plot.bottom1 - ((y - options.y1Min) / (options.y1Max - options.y1Min)) * (plot.bottom1 - plot.top1);
      const sy2 = (y) => plot.bottom2 - ((y - options.y2Min) / (options.y2Max - options.y2Min)) * (plot.bottom2 - plot.top2);
      const drawFrame = (top, bottom, label) => {
        ctx.strokeStyle = "#dbe7ea";
        ctx.lineWidth = 1;
        for (let i = 0; i <= 4; i += 1) {
          const y = top + ((bottom - top) * i) / 4;
          ctx.beginPath();
          ctx.moveTo(plot.left, y);
          ctx.lineTo(plot.right, y);
          ctx.stroke();
        }
        ctx.strokeStyle = "#172d33";
        ctx.strokeRect(plot.left, top, plot.right - plot.left, bottom - top);
        ctx.fillStyle = "#172d33";
        ctx.font = "13px sans-serif";
        ctx.save();
        ctx.translate(18, (top + bottom) / 2);
        ctx.rotate(-Math.PI / 2);
        ctx.fillText(label, 0, 0);
        ctx.restore();
      };
      drawFrame(plot.top1, plot.bottom1, options.y1Label);
      drawFrame(plot.top2, plot.bottom2, options.y2Label);
      ctx.fillStyle = "#526047";
      ctx.font = "12px sans-serif";
      ctx.fillText(options.xLabel || "frequency (rad/s)", plot.right - 128, height - 14);
      if (options.regions) {
        for (const region of options.regions) {
          const x1 = Math.max(plot.left, sx(region.x1));
          const x2 = Math.min(plot.right, sx(region.x2));
          if (!Number.isFinite(x1) || !Number.isFinite(x2) || x2 <= x1) continue;
          ctx.fillStyle = region.color || "rgba(213, 107, 53, 0.12)";
          ctx.fillRect(x1, plot.top1, x2 - x1, plot.bottom1 - plot.top1);
          ctx.fillRect(x1, plot.top2, x2 - x1, plot.bottom2 - plot.top2);
          if (region.label) {
            ctx.fillStyle = region.textColor || "#6f4f20";
            ctx.font = "12px sans-serif";
            ctx.fillText(region.label, x1 + 8, plot.bottom2 - 12);
          }
        }
      }
      const drawSeries = (points, yScale, color, dash = [], clip) => {
        ctx.save();
        if (clip) {
          ctx.beginPath();
          ctx.rect(plot.left, clip.top, plot.right - plot.left, clip.bottom - clip.top);
          ctx.clip();
        }
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.1;
        ctx.setLineDash(dash);
        ctx.beginPath();
        points.forEach((point, i) => {
          const x = sx(point.x);
          const y = yScale(point.y);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.restore();
      };
      for (const item of series.top) drawSeries(item.points, sy1, item.color, item.dash, { top: plot.top1, bottom: plot.bottom1 });
      for (const item of series.bottom) drawSeries(item.points, sy2, item.color, item.dash, { top: plot.top2, bottom: plot.bottom2 });
      const markerList = options.markers || (options.marker ? [{ x: options.marker }] : []);
      for (const marker of markerList) {
        const x = sx(marker.x);
        if (!Number.isFinite(x)) continue;
        ctx.strokeStyle = marker.color || "#8b5e24";
        ctx.setLineDash(marker.dash || [5, 5]);
        ctx.beginPath();
        ctx.moveTo(x, plot.top1);
        ctx.lineTo(x, plot.bottom2);
        ctx.stroke();
        ctx.setLineDash([]);
        if (marker.label) {
          ctx.fillStyle = marker.color || "#8b5e24";
          ctx.font = "12px sans-serif";
          ctx.fillText(marker.label, Math.min(x + 6, plot.right - 94), plot.top1 + 14);
        }
      }
      if (options.legend) {
        const legendHeight = 16 + 20 * options.legend.length;
        ctx.fillStyle = "rgba(255,255,255,0.92)";
        ctx.strokeStyle = "#dbe7ea";
        drawCanvasRoundedRect(ctx, width - 230, 28, 196, legendHeight, 8);
        ctx.font = "13px sans-serif";
        options.legend.forEach((item, i) => {
          ctx.strokeStyle = item.color;
          ctx.setLineDash(item.dash || []);
          ctx.beginPath();
          ctx.moveTo(width - 214, 50 + 20 * i);
          ctx.lineTo(width - 184, 50 + 20 * i);
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.fillStyle = "#172d33";
          ctx.fillText(item.label, width - 176, 54 + 20 * i);
        });
      }
    };
    const drawCh15SineResponse = (activity, omega) => {
      const canvas = activity.querySelector("[data-plot='ch15-sine-response']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);
      const mag = 1 / Math.sqrt(1 + omega * omega);
      const phase = -Math.atan(omega);
      const tEnd = 70;
      const points = Array.from({ length: 500 }, (_, i) => {
        const t = (tEnd * i) / 499;
        return { t, u: Math.sin(omega * t), y: mag * Math.sin(omega * t + phase) };
      });
      const plot = { left: 58, right: width - 24, top: 30, bottom: height - 44 };
      const sx = (t) => plot.left + (t / tEnd) * (plot.right - plot.left);
      const sy = (v) => plot.bottom - ((v + 1.1) / 2.2) * (plot.bottom - plot.top);
      ctx.strokeStyle = "#dbe7ea";
      for (let i = 0; i <= 4; i += 1) {
        const y = plot.top + ((plot.bottom - plot.top) * i) / 4;
        ctx.beginPath();
        ctx.moveTo(plot.left, y);
        ctx.lineTo(plot.right, y);
        ctx.stroke();
      }
      ctx.strokeStyle = "#172d33";
      ctx.strokeRect(plot.left, plot.top, plot.right - plot.left, plot.bottom - plot.top);
      const trace = (key, color, dash = []) => {
        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.2;
        ctx.setLineDash(dash);
        ctx.beginPath();
        points.forEach((point, i) => {
          const x = sx(point.t);
          const y = sy(point[key]);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.restore();
      };
      trace("u", "#7f8d93", [7, 5]);
      trace("y", "#0e6d77");
      ctx.fillStyle = "#172d33";
      ctx.font = "13px sans-serif";
      ctx.fillText("time (s)", plot.right - 58, height - 16);
      ctx.fillStyle = "rgba(255,255,255,0.92)";
      ctx.strokeStyle = "#dbe7ea";
      drawCanvasRoundedRect(ctx, width - 214, 36, 176, 56, 8);
      ctx.fillStyle = "#7f8d93";
      ctx.fillText("dashed: input u(t)", width - 196, 58);
      ctx.fillStyle = "#0e6d77";
      ctx.fillText("solid: output y(t)", width - 196, 80);
    };
    const drawCh15FrequencyScale = (activity, omega) => {
      const canvas = activity.querySelector("[data-plot='ch15-frequency-scale']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      const w = logspace(-2, 1.3, 360);
      const mag = w.map((x) => 1 / Math.sqrt(1 + x * x));
      const db = mag.map((m) => 20 * Math.log10(m));
      const phase = w.map((x) => -Math.atan(x) * 180 / Math.PI);
      const selectedMag = 1 / Math.sqrt(1 + omega * omega);
      const selectedDb = 20 * Math.log10(selectedMag);
      const selectedPhase = -Math.atan(omega) * 180 / Math.PI;
      const colors = { mag: "#0e6d77", phase: "#d56b35", marker: "#8b5e24", corner: "#526047" };
      const panels = {
        linMag: { left: 68, right: width * 0.48, top: 72, bottom: 230, title: "Linear scale", yLabel: "|H(jw)|", yMin: 0, yMax: 1.05 },
        linPhase: { left: 68, right: width * 0.48, top: 306, bottom: 464, title: "", yLabel: "phase (deg)", yMin: -95, yMax: 5 },
        bodeMag: { left: width * 0.57, right: width - 42, top: 72, bottom: 230, title: "Bode scale", yLabel: "dB", yMin: -28, yMax: 3 },
        bodePhase: { left: width * 0.57, right: width - 42, top: 306, bottom: 464, title: "", yLabel: "phase (deg)", yMin: -95, yMax: 5 },
      };
      const linXMin = 0;
      const linXMax = 20;
      const logXMin = 0.01;
      const logXMax = 20;
      const linX = (panel, x) => panel.left + ((x - linXMin) / (linXMax - linXMin)) * (panel.right - panel.left);
      const logX = (panel, x) => panel.left + ((Math.log10(x) - Math.log10(logXMin)) / (Math.log10(logXMax) - Math.log10(logXMin))) * (panel.right - panel.left);
      const yScale = (panel, y) => panel.bottom - ((y - panel.yMin) / (panel.yMax - panel.yMin)) * (panel.bottom - panel.top);
      const clipOmega = Math.min(linXMax, Math.max(linXMin, omega));
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);
      const drawPanel = (panel, xScale, xTicks, yTicks) => {
        ctx.save();
        ctx.strokeStyle = "#dbe7ea";
        ctx.lineWidth = 1;
        for (const tick of yTicks) {
          const y = yScale(panel, tick);
          ctx.beginPath();
          ctx.moveTo(panel.left, y);
          ctx.lineTo(panel.right, y);
          ctx.stroke();
          ctx.fillStyle = "#526047";
          ctx.font = "11px sans-serif";
          ctx.textAlign = "right";
          ctx.fillText(String(tick), panel.left - 8, y + 4);
        }
        for (const tick of xTicks) {
          const x = xScale(panel, tick);
          ctx.beginPath();
          ctx.moveTo(x, panel.top);
          ctx.lineTo(x, panel.bottom);
          ctx.stroke();
          ctx.fillStyle = "#526047";
          ctx.font = "11px sans-serif";
          ctx.textAlign = "center";
          ctx.fillText(tick < 1 ? String(tick) : String(Math.round(tick)), x, panel.bottom + 18);
        }
        ctx.strokeStyle = "#172d33";
        ctx.strokeRect(panel.left, panel.top, panel.right - panel.left, panel.bottom - panel.top);
        ctx.fillStyle = "#172d33";
        ctx.font = "700 15px sans-serif";
        ctx.textAlign = "center";
        if (panel.title) ctx.fillText(panel.title, (panel.left + panel.right) / 2, panel.top - 42);
        ctx.save();
        ctx.translate(panel.left - 46, (panel.top + panel.bottom) / 2);
        ctx.rotate(-Math.PI / 2);
        ctx.font = "13px sans-serif";
        ctx.fillText(panel.yLabel, 0, 0);
        ctx.restore();
        ctx.restore();
      };
      const drawTrace = (panel, xScale, values, color) => {
        ctx.save();
        ctx.beginPath();
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.1;
        w.forEach((freq, i) => {
          const x = xScale(panel, freq);
          const y = yScale(panel, values[i]);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.restore();
      };
      const drawVertical = (panel, x, color, dash = []) => {
        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.5;
        ctx.setLineDash(dash);
        ctx.beginPath();
        ctx.moveTo(x, panel.top);
        ctx.lineTo(x, panel.bottom);
        ctx.stroke();
        ctx.restore();
      };
      const drawPoint = (panel, x, y, color, label, align = "left") => {
        ctx.save();
        ctx.fillStyle = color;
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = "rgba(255,255,255,0.94)";
        ctx.strokeStyle = "#dbe7ea";
        const labelWidth = 150;
        const boxX = align === "left" ? Math.min(x + 10, panel.right - labelWidth - 4) : Math.max(x - labelWidth - 10, panel.left + 4);
        const boxY = Math.max(panel.top + 8, Math.min(y - 26, panel.bottom - 34));
        drawCanvasRoundedRect(ctx, boxX, boxY, labelWidth, 26, 6);
        ctx.fillStyle = "#172d33";
        ctx.font = "12px sans-serif";
        ctx.textAlign = "left";
        ctx.fillText(label, boxX + 8, boxY + 17);
        ctx.restore();
      };
      const linXTicks = [0, 5, 10, 15, 20];
      const logXTicks = [0.01, 0.1, 1, 10];
      drawPanel(panels.linMag, linX, linXTicks, [0, 0.25, 0.5, 0.75, 1]);
      drawPanel(panels.linPhase, linX, linXTicks, [-90, -60, -30, 0]);
      drawPanel(panels.bodeMag, logX, logXTicks, [-20, -10, 0]);
      drawPanel(panels.bodePhase, logX, logXTicks, [-90, -60, -30, 0]);
      ctx.fillStyle = "rgba(213, 107, 53, 0.09)";
      ctx.fillRect(panels.linMag.left, panels.linMag.top, linX(panels.linMag, 1) - panels.linMag.left, panels.linMag.bottom - panels.linMag.top);
      ctx.fillRect(panels.linPhase.left, panels.linPhase.top, linX(panels.linPhase, 1) - panels.linPhase.left, panels.linPhase.bottom - panels.linPhase.top);
      ctx.fillStyle = "#9c4735";
      ctx.font = "12px sans-serif";
      ctx.textAlign = "left";
      ctx.fillText("0 to 1 rad/s compressed", panels.linMag.left + 8, panels.linMag.top + 18);
      drawTrace(panels.linMag, linX, mag, colors.mag);
      drawTrace(panels.linPhase, linX, phase, colors.phase);
      drawTrace(panels.bodeMag, logX, db, colors.mag);
      drawTrace(panels.bodePhase, logX, phase, colors.phase);
      for (const panel of Object.values(panels)) {
        const xCorner = panel === panels.linMag || panel === panels.linPhase ? linX(panel, 1) : logX(panel, 1);
        drawVertical(panel, xCorner, colors.corner, [3, 4]);
      }
      ctx.fillStyle = colors.corner;
      ctx.font = "12px sans-serif";
      ctx.textAlign = "center";
      const cornerLabel = "corner: omega = 1, -3.01 dB, -45 deg";
      const cornerX = (panels.bodeMag.left + panels.bodeMag.right) / 2;
      const cornerY = panels.bodeMag.top - 16;
      const cornerWidth = ctx.measureText(cornerLabel).width + 18;
      ctx.fillStyle = "rgba(255,255,255,0.94)";
      ctx.strokeStyle = "#dbe7ea";
      drawCanvasRoundedRect(ctx, cornerX - cornerWidth / 2, cornerY - 15, cornerWidth, 22, 6);
      ctx.fillStyle = colors.corner;
      ctx.fillText(cornerLabel, cornerX, cornerY);
      const linMarkerX = linX(panels.linMag, clipOmega);
      const linMarkerX2 = linX(panels.linPhase, clipOmega);
      const bodeMarkerX = logX(panels.bodeMag, omega);
      const bodeMarkerX2 = logX(panels.bodePhase, omega);
      drawVertical(panels.linMag, linMarkerX, colors.marker, [6, 5]);
      drawVertical(panels.linPhase, linMarkerX2, colors.marker, [6, 5]);
      drawVertical(panels.bodeMag, bodeMarkerX, colors.marker, [6, 5]);
      drawVertical(panels.bodePhase, bodeMarkerX2, colors.marker, [6, 5]);
      drawPoint(panels.linMag, linMarkerX, yScale(panels.linMag, selectedMag), colors.mag, `omega ${fmt(omega, 3)}, |H| ${fmt(selectedMag, 3)}`);
      drawPoint(panels.linPhase, linMarkerX2, yScale(panels.linPhase, selectedPhase), colors.phase, `phase ${fmt(selectedPhase, 1)} deg`);
      drawPoint(panels.bodeMag, bodeMarkerX, yScale(panels.bodeMag, selectedDb), colors.mag, `${fmt(selectedDb, 2)} dB`, "right");
      drawPoint(panels.bodePhase, bodeMarkerX2, yScale(panels.bodePhase, selectedPhase), colors.phase, `phase ${fmt(selectedPhase, 1)} deg`, "right");
      ctx.fillStyle = "#526047";
      ctx.font = "12px sans-serif";
      ctx.textAlign = "center";
      ctx.fillText("frequency omega (rad/s)", (panels.linPhase.left + panels.linPhase.right) / 2, height - 32);
      ctx.fillText("frequency omega (rad/s, log scale)", (panels.bodePhase.left + panels.bodePhase.right) / 2, height - 32);
    };
    const poleZeroResponse = (element, corner, w) => {
      const isZero = element.startsWith("zero");
      const isRhp = element.endsWith("rhp");
      const ratio = w / corner;
      if (isZero) {
        return {
          magDb: 10 * Math.log10(1 + ratio * ratio),
          phase: isRhp ? 180 - Math.atan(ratio) * 180 / Math.PI : Math.atan(ratio) * 180 / Math.PI,
        };
      }
      return {
        magDb: -10 * Math.log10(1 + ratio * ratio),
        phase: isRhp ? Math.atan(ratio) * 180 / Math.PI : -Math.atan(ratio) * 180 / Math.PI,
      };
    };
    const poleZeroApprox = (element, corner, w) => {
      const isZero = element.startsWith("zero");
      const isRhp = element.endsWith("rhp");
      const phaseStart = isZero && isRhp ? 180 : 0;
      const phaseEnd = isZero ? 90 : (isRhp ? 90 : -90);
      const logRatio = Math.log10(w / corner);
      const magDb = isZero
        ? Math.max(0, 20 * logRatio)
        : Math.min(0, -20 * logRatio);
      let phase;
      if (logRatio <= -1) phase = phaseStart;
      else if (logRatio >= 1) phase = phaseEnd;
      else phase = phaseStart + ((logRatio + 1) / 2) * (phaseEnd - phaseStart);
      return { magDb, phase };
    };
    const drawCh15PoleZeroBuilder = (activity, element, corner) => {
      const canvas = activity.querySelector("[data-plot='ch15-pole-zero-builder']");
      const w = logspace(-2, 2, 420);
      const exact = w.map((x) => ({ x, ...poleZeroResponse(element, corner, x) }));
      const approx = w.map((x) => ({ x, ...poleZeroApprox(element, corner, x) }));
      const phaseVals = exact.map((p) => p.phase).concat(approx.map((p) => p.phase));
      drawTwoPanelPlot(canvas, {
        top: [
          { points: exact.map((p) => ({ x: p.x, y: p.magDb })), color: "#0e6d77" },
          { points: approx.map((p) => ({ x: p.x, y: p.magDb })), color: "#172d33", dash: [7, 5] },
        ],
        bottom: [
          { points: exact.map((p) => ({ x: p.x, y: p.phase })), color: "#d56b35" },
          { points: approx.map((p) => ({ x: p.x, y: p.phase })), color: "#172d33", dash: [7, 5] },
        ],
      }, {
        xMin: 0.01,
        xMax: 100,
        y1Min: -45,
        y1Max: 45,
        y2Min: Math.min(-105, Math.floor(Math.min(...phaseVals) / 30) * 30),
        y2Max: Math.max(195, Math.ceil(Math.max(...phaseVals) / 30) * 30),
        y1Label: "magnitude (dB)",
        y2Label: "phase (deg)",
        marker: corner,
        legend: [
          { label: "exact", color: "#0e6d77" },
          { label: "straight-line", color: "#172d33", dash: [7, 5] },
        ],
      });
    };
    const drawCh15SecondOrderBode = (activity, zeta, wn) => {
      const canvas = activity.querySelector("[data-plot='ch15-second-order-bode']");
      const w = logspace(-2, 2, 420);
      const response = w.map((x) => {
        const r = x / wn;
        const real = 1 - r * r;
        const imag = 2 * zeta * r;
        const den = Math.hypot(real, imag);
        return { x, magDb: 20 * Math.log10(1 / den), phase: -Math.atan2(imag, real) * 180 / Math.PI };
      });
      const phase = unwrapDegrees(response.map((p) => p.phase));
      drawTwoPanelPlot(canvas, {
        top: [{ points: response.map((p) => ({ x: p.x, y: p.magDb })), color: "#0e6d77" }],
        bottom: [{ points: response.map((p, i) => ({ x: p.x, y: phase[i] })), color: "#d56b35" }],
      }, {
        xMin: 0.01,
        xMax: 100,
        y1Min: -70,
        y1Max: 25,
        y2Min: -190,
        y2Max: 10,
        y1Label: "magnitude (dB)",
        y2Label: "phase (deg)",
        marker: wn,
      });
    };
    const ch15ExampleTerms = {
      example1: [
        { label: "gain 2", type: "gain", value: 2 },
        { label: "integrator 1/s", type: "integrator" },
        { label: "pole at 10", type: "pole", corner: 10 },
      ],
      example2: [
        { label: "gain 10", type: "gain", value: 10 },
        { label: "zero at 1", type: "zero", corner: 1 },
        { label: "pole at 2", type: "pole", corner: 2 },
        { label: "zero at 10", type: "zero", corner: 10 },
        { label: "pole at 20", type: "pole", corner: 20 },
        { label: "pole at 200", type: "pole", corner: 200 },
      ],
    };
    const bodeTermExact = (term, w) => {
      if (term.type === "gain") return { magDb: 20 * Math.log10(term.value), phase: 0 };
      if (term.type === "integrator") return { magDb: -20 * Math.log10(w), phase: -90 };
      const ratio = w / term.corner;
      const magDb = (term.type === "zero" ? 1 : -1) * 10 * Math.log10(1 + ratio * ratio);
      const phase = (term.type === "zero" ? 1 : -1) * Math.atan(ratio) * 180 / Math.PI;
      return { magDb, phase };
    };
    const bodeTermApprox = (term, w) => {
      if (term.type === "gain") return { magDb: 20 * Math.log10(term.value), phase: 0 };
      if (term.type === "integrator") return { magDb: -20 * Math.log10(w), phase: -90 };
      const logRatio = Math.log10(w / term.corner);
      const magDb = term.type === "zero" ? Math.max(0, 20 * logRatio) : Math.min(0, -20 * logRatio);
      const finalPhase = term.type === "zero" ? 90 : -90;
      let phase;
      if (logRatio <= -1) phase = 0;
      else if (logRatio >= 1) phase = finalPhase;
      else phase = ((logRatio + 1) / 2) * finalPhase;
      return { magDb, phase };
    };
    const ch15Example1Terms = [
      { key: "gain", input: "ex1-gain", label: "gain 2", type: "gain", value: 2, lesson: "Gain raises the whole magnitude plot by 6.02 dB and does not change phase." },
      { key: "integrator", input: "ex1-integrator", label: "integrator 1/s", type: "integrator", lesson: "The integrator adds a -20 dB/dec slope everywhere and contributes -90 deg of phase." },
      { key: "pole", input: "ex1-pole", label: "pole at 10", type: "pole", corner: 10, lesson: "The pole at omega = 10 adds another -20 dB/dec after its corner and transitions phase toward -90 deg." },
    ];
    const sumBodeTerms = (terms, w, evaluator) => terms.reduce((sum, term) => {
      const value = evaluator(term, w);
      return { magDb: sum.magDb + value.magDb, phase: sum.phase + value.phase };
    }, { magDb: 0, phase: 0 });
    const isInputChecked = (activity, key) => Boolean(activity.querySelector(`[data-input="${key}"]`)?.checked);
    const describeExample1Lesson = (active) => {
      if (!active.length) return "Turn on a term to see how it contributes to the Bode approximation.";
      const last = active[active.length - 1];
      const slope = active.some((term) => term.key === "integrator") ? -20 : 0;
      const finalSlope = slope + (active.some((term) => term.key === "pole") ? -20 : 0);
      return `${last.lesson} Current high-frequency straight-line slope: ${finalSlope} dB/dec.`;
    };
    const exampleFinalBehavior = (terms) => {
      const slope = terms.reduce((sum, term) => {
        if (term.type === "integrator") return sum - 20;
        if (term.type === "pole") return sum - 20;
        if (term.type === "zero") return sum + 20;
        return sum;
      }, 0);
      const phase = terms.reduce((sum, term) => {
        if (term.type === "integrator") return sum - 90;
        if (term.type === "pole") return sum - 90;
        if (term.type === "zero") return sum + 90;
        return sum;
      }, 0);
      return { slope, phase };
    };
    const describeExampleBuilderLesson = (included, terms) => {
      if (!included.length) return "Turn on terms to build the Bode approximation by graphical addition.";
      const final = exampleFinalBehavior(included);
      const remaining = terms.length - included.length;
      const last = included[included.length - 1];
      const remainingText = remaining ? `${remaining} term${remaining === 1 ? "" : "s"} still not included.` : "All terms are included.";
      return `${last.label} is included. Current high-frequency slope: ${final.slope} dB/dec and phase: ${final.phase} deg. ${remainingText}`;
    };
    const drawCh15Example1Builder = (activity, omegaExp) => {
      const canvas = activity.querySelector("[data-plot='ch15-example-1-builder']");
      const active = ch15Example1Terms.filter((term) => isInputChecked(activity, term.input));
      const omega = 10 ** omegaExp;
      const w = logspace(-2, 3, 460);
      const exactTerms = ch15ExampleTerms.example1;
      const exact = w.map((x) => ({ x, ...sumBodeTerms(exactTerms, x, bodeTermExact) }));
      const approx = w.map((x) => ({ x, ...sumBodeTerms(active, x, bodeTermApprox) }));
      const termColor = "#9aa7ad";
      const individualTop = active.map((term, index) => ({
        points: w.map((x) => ({ x, y: bodeTermApprox(term, x).magDb })),
        color: termColor,
        dash: index % 2 ? [3, 5] : [8, 5],
      }));
      const individualBottom = active.map((term, index) => ({
        points: w.map((x) => ({ x, y: bodeTermApprox(term, x).phase })),
        color: termColor,
        dash: index % 2 ? [3, 5] : [8, 5],
      }));
      drawTwoPanelPlot(canvas, {
        top: [
          ...individualTop,
          { points: exact.map((p) => ({ x: p.x, y: p.magDb })), color: "#0e6d77" },
          { points: approx.map((p) => ({ x: p.x, y: p.magDb })), color: "#172d33", dash: [7, 5] },
        ],
        bottom: [
          ...individualBottom,
          { points: exact.map((p) => ({ x: p.x, y: p.phase })), color: "#d56b35" },
          { points: approx.map((p) => ({ x: p.x, y: p.phase })), color: "#172d33", dash: [7, 5] },
        ],
      }, {
        xMin: 0.01,
        xMax: 1000,
        y1Min: -80,
        y1Max: 60,
        y2Min: -210,
        y2Max: 20,
        y1Label: "magnitude (dB)",
        y2Label: "phase (deg)",
        markers: [
          { x: 10, label: "pole corner", color: "#6f7f32", dash: [4, 4] },
          { x: omega, label: `omega = ${fmt(omega, 2)}`, color: "#8b5e24", dash: [5, 5] },
        ],
        legend: [
          { label: "individual terms", color: termColor, dash: [8, 5] },
          { label: "exact full", color: "#0e6d77" },
          { label: "cumulative approx", color: "#172d33", dash: [7, 5] },
        ],
      });
      const exactProbe = sumBodeTerms(exactTerms, omega, bodeTermExact);
      const approxProbe = sumBodeTerms(active, omega, bodeTermApprox);
      return { active, omega, exactProbe, approxProbe };
    };
    const updateCh15ExampleTermCheckboxes = (activity, terms, exampleKey) => {
      if (activity.dataset.exampleBuilderKey !== exampleKey) {
        for (let index = 0; index < 6; index += 1) {
          const input = activity.querySelector(`[data-input="example-term-${index}"]`);
          if (input) input.checked = true;
        }
        activity.dataset.exampleBuilderKey = exampleKey;
      }
      for (let index = 0; index < 6; index += 1) {
        const input = activity.querySelector(`[data-input="example-term-${index}"]`);
        const label = input?.closest(".ch15-example-term");
        const text = activity.querySelector(`[data-output="example-term-label-${index}"]`);
        const term = terms[index];
        if (label) label.hidden = !term;
        if (text) text.textContent = term ? term.label : "";
        if (input) input.disabled = !term;
      }
    };
    const drawCh15BodeExampleBuilder = (activity, exampleKey, omegaExp) => {
      const canvas = activity.querySelector("[data-plot='ch15-bode-example-builder']");
      const terms = ch15ExampleTerms[exampleKey] || ch15ExampleTerms.example1;
      updateCh15ExampleTermCheckboxes(activity, terms, exampleKey);
      const included = terms.filter((_, index) => isInputChecked(activity, `example-term-${index}`));
      const omega = 10 ** omegaExp;
      const w = logspace(-2, 3, 460);
      const exact = w.map((x) => ({ x, ...sumBodeTerms(terms, x, bodeTermExact) }));
      const approx = w.map((x) => ({ x, ...sumBodeTerms(included, x, bodeTermApprox) }));
      const termColor = "#9aa7ad";
      const individualTop = included.map((term, index) => ({
        points: w.map((x) => ({ x, y: bodeTermApprox(term, x).magDb })),
        color: termColor,
        dash: index % 2 ? [3, 5] : [8, 5],
      }));
      const individualBottom = included.map((term, index) => ({
        points: w.map((x) => ({ x, y: bodeTermApprox(term, x).phase })),
        color: termColor,
        dash: index % 2 ? [3, 5] : [8, 5],
      }));
      const cornerMarkers = terms
        .filter((term) => term.corner)
        .map((term) => ({ x: term.corner, label: term.label, color: "#6f7f32", dash: [4, 4] }));
      drawTwoPanelPlot(canvas, {
        top: [
          ...individualTop,
          { points: exact.map((p) => ({ x: p.x, y: p.magDb })), color: "#0e6d77" },
          { points: approx.map((p) => ({ x: p.x, y: p.magDb })), color: "#172d33", dash: [7, 5] },
        ],
        bottom: [
          ...individualBottom,
          { points: exact.map((p) => ({ x: p.x, y: p.phase })), color: "#d56b35" },
          { points: approx.map((p) => ({ x: p.x, y: p.phase })), color: "#172d33", dash: [7, 5] },
        ],
      }, {
        xMin: 0.01,
        xMax: 1000,
        y1Min: -80,
        y1Max: 60,
        y2Min: -260,
        y2Max: 120,
        y1Label: "magnitude (dB)",
        y2Label: "phase (deg)",
        markers: [
          ...cornerMarkers,
          { x: omega, label: `omega = ${fmt(omega, 2)}`, color: "#8b5e24", dash: [5, 5] },
        ],
        legend: [
          { label: "individual terms", color: termColor, dash: [8, 5] },
          { label: "exact full", color: "#0e6d77" },
          { label: "cumulative approx", color: "#172d33", dash: [7, 5] },
        ],
      });
      const exactProbe = sumBodeTerms(terms, omega, bodeTermExact);
      const approxProbe = sumBodeTerms(included, omega, bodeTermApprox);
      return { included, terms, total: terms.length, omega, exactProbe, approxProbe };
    };
    const ch18BlockResponse = (block, corner, mRaw, w) => {
      const m = Math.max(1.01, mRaw);
      const ratio = w / corner;
      if (block === "gain") {
        return { magDb: 20 * Math.log10(corner), phase: 0 };
      }
      if (block === "pi") {
        return { magDb: 10 * Math.log10(1 + (corner / w) ** 2), phase: -Math.atan(corner / w) * 180 / Math.PI };
      }
      if (block === "lpf") {
        return { magDb: -10 * Math.log10(1 + ratio * ratio), phase: -Math.atan(ratio) * 180 / Math.PI };
      }
      if (block === "lag") {
        const pole = corner / m;
        return {
          magDb: 10 * Math.log10((w * w + corner * corner) / (w * w + pole * pole)),
          phase: Math.atan(w / corner) * 180 / Math.PI - Math.atan(w / pole) * 180 / Math.PI,
        };
      }
      const z = corner / Math.sqrt(m);
      const p = corner * Math.sqrt(m);
      return {
        magDb: 20 * Math.log10(m) + 10 * Math.log10((w * w + z * z) / (w * w + p * p)),
        phase: Math.atan(w / z) * 180 / Math.PI - Math.atan(w / p) * 180 / Math.PI,
      };
    };
    const ch18BlockDetails = (block, corner, mRaw) => {
      const m = Math.max(1.01, mRaw);
      const dbM = 20 * Math.log10(m);
      if (block === "gain") {
        return {
          transfer: `C(s) = ${fmt(corner, 2)}`,
          cornerLabel: "Gain value",
          cornerValue: `K = ${fmt(corner, 3)} (${fmt(20 * Math.log10(corner), 2)} dB)`,
          ratioLabel: "Ratio M",
          mValue: "",
          derived: "No poles or zeros are added by a positive proportional gain.",
          hint: "Gain shifts the magnitude curve up or down at every frequency and does not change phase.",
          magEffect: "Moves the entire magnitude plot up or down by a constant amount.",
          phaseEffect: "Adds no phase when the gain is positive.",
          use: "moving the gain crossover frequency",
          warning: "more gain usually increases bandwidth and control effort",
          regionText: "all frequencies",
          markers: [],
          regions: [{ x1: 0.001, x2: 1000, label: "all frequencies affected", color: "rgba(14, 109, 119, 0.08)", textColor: "#0e6d77" }],
        };
      }
      if (block === "pi") {
        return {
          transfer: `C(s) = (s + ${fmt(corner, 2)}) / s`,
          cornerLabel: "PI zero location",
          cornerValue: `z<sub>PI</sub> = ${fmt(corner, 3)} rad/s`,
          ratioLabel: "Ratio M",
          mValue: "",
          derived: `Integrator pole at s = 0; zero at s = -${fmt(corner, 3)}.`,
          hint: "PI control raises low-frequency gain and changes system type, but it adds phase lag near the PI zero.",
          magEffect: "Raises low-frequency gain and changes the system type.",
          phaseEffect: "Adds phase lag below and around the integral corner.",
          use: "improving tracking and disturbance rejection at low frequency",
          warning: "keep the corner below crossover to protect phase margin",
          regionText: `below k_I = ${fmt(corner, 3)} rad/s`,
          markers: [{ x: corner, label: "k_I", color: "#6f7f32", dash: [4, 4] }],
          regions: [{ x1: 0.001, x2: corner, label: "low-frequency boost", color: "rgba(111, 127, 50, 0.12)", textColor: "#526047" }],
        };
      }
      if (block === "lpf") {
        return {
          transfer: `C(s) = ${fmt(corner, 2)} / (s + ${fmt(corner, 2)})`,
          cornerLabel: "Low-pass pole",
          cornerValue: `p = ${fmt(corner, 3)} rad/s`,
          ratioLabel: "Ratio M",
          mValue: "",
          derived: `Pole at s = -${fmt(corner, 3)}; DC gain is one.`,
          hint: "A low-pass filter reduces high-frequency gain for noise attenuation, but it adds phase lag near and above the pole.",
          magEffect: "Leaves low frequencies unchanged and rolls off high frequencies.",
          phaseEffect: "Adds phase lag near and above the pole.",
          use: "attenuating high-frequency sensor noise",
          warning: "placing the pole near crossover can reduce phase margin",
          regionText: `above p = ${fmt(corner, 3)} rad/s`,
          markers: [{ x: corner, label: "pole", color: "#9c4735", dash: [4, 4] }],
          regions: [{ x1: corner, x2: 1000, label: "noise attenuation", color: "rgba(156, 71, 53, 0.11)", textColor: "#9c4735" }],
        };
      }
      if (block === "lag") {
        const pole = corner / m;
        const zero = corner;
        return {
          transfer: `C(s) = (s + ${fmt(zero, 2)}) / (s + ${fmt(pole, 3)})`,
          cornerLabel: "Lag zero location",
          cornerValue: `z<sub>lag</sub> = ${fmt(zero, 3)} rad/s`,
          ratioLabel: "Lag ratio",
          mValue: `M = ${fmt(m, 1)}`,
          derived: `Pole p_lag = z_lag/M = ${fmt(pole, 3)} rad/s; low-frequency boost is about ${fmt(dbM, 1)} dB.`,
          hint: "Lag increases low-frequency gain without changing high-frequency gain, but it adds phase lag between the pole and zero.",
          magEffect: "Raises low-frequency gain without changing the final high-frequency level.",
          phaseEffect: "Adds negative phase between the pole and zero.",
          use: "improving low-frequency tracking/rejection while preserving system type",
          warning: "place lag well below crossover so the phase dip does not hurt stability",
          regionText: `phase lag from p = ${fmt(pole, 3)} to z = ${fmt(zero, 3)} rad/s`,
          markers: [
            { x: pole, label: "pole", color: "#9c4735", dash: [4, 4] },
            { x: zero, label: "zero", color: "#0e6d77", dash: [4, 4] },
          ],
          regions: [{ x1: pole, x2: zero, label: "phase lag region", color: "rgba(213, 107, 53, 0.12)", textColor: "#9c4735" }],
        };
      }
      const zero = corner / Math.sqrt(m);
      const pole = corner * Math.sqrt(m);
      return {
        transfer: `C(s) = ${fmt(m, 1)}(s + ${fmt(zero, 3)}) / (s + ${fmt(pole, 3)})`,
        cornerLabel: "Lead center frequency",
        cornerValue: `&omega;<sub>lead</sub> = ${fmt(corner, 3)} rad/s`,
        ratioLabel: "Lead ratio",
        mValue: `M = ${fmt(m, 1)}`,
        derived: `Zero z_lead = ${fmt(zero, 3)} rad/s; pole p_lead = ${fmt(pole, 3)} rad/s.`,
        hint: "Lead adds positive phase near the center frequency, but it also increases high-frequency gain.",
        magEffect: `Raises high-frequency gain by about ${fmt(dbM, 1)} dB.`,
        phaseEffect: "Adds positive phase between the zero and pole, peaking near the center frequency.",
        use: "adding phase near crossover to improve phase margin",
        warning: "the high-frequency gain increase can amplify sensor noise",
        regionText: `phase lead from z = ${fmt(zero, 3)} to p = ${fmt(pole, 3)} rad/s`,
        markers: [
          { x: zero, label: "zero", color: "#0e6d77", dash: [4, 4] },
          { x: corner, label: "center", color: "#8b5e24", dash: [5, 5] },
          { x: pole, label: "pole", color: "#9c4735", dash: [4, 4] },
        ],
        regions: [{ x1: zero, x2: pole, label: "phase lead region", color: "rgba(14, 109, 119, 0.11)", textColor: "#0e6d77" }],
      };
    };
    const drawCh18BlockExplorer = (activity, block, cornerExp, m, probeExp) => {
      const canvas = activity.querySelector("[data-plot='ch18-block-explorer']");
      const corner = 10 ** cornerExp;
      const probe = 10 ** probeExp;
      const details = ch18BlockDetails(block, corner, m);
      const w = logspace(-3, 3, 520);
      const response = w
        .map((x) => ({ x, ...ch18BlockResponse(block, corner, m, x) }))
        .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.magDb) && Number.isFinite(point.phase));
      const phaseValues = response.map((p) => p.phase);
      drawTwoPanelPlot(canvas, {
        top: [{ points: response.map((p) => ({ x: p.x, y: p.magDb })), color: "#0e6d77" }],
        bottom: [{ points: response.map((p) => ({ x: p.x, y: p.phase })), color: "#d56b35" }],
      }, {
        xMin: 0.001,
        xMax: 1000,
        y1Min: -45,
        y1Max: 45,
        y2Min: Math.min(-100, Math.floor(Math.min(...phaseValues) / 20) * 20),
        y2Max: Math.max(100, Math.ceil(Math.max(...phaseValues) / 20) * 20),
        y1Label: "magnitude (dB)",
        y2Label: "phase (deg)",
        regions: details.regions,
        markers: [
          ...details.markers,
          { x: probe, label: `probe ${fmt(probe, 2)}`, color: "#172d33", dash: [6, 5] },
        ],
        legend: [
          { label: "magnitude", color: "#0e6d77" },
          { label: "phase", color: "#d56b35" },
        ],
      });
      return { details, corner, probe, probeResponse: ch18BlockResponse(block, corner, m, probe) };
    };
    const ch18LoopshapeSteps = [
      {
        added: "Plant only",
        block: "No compensator block yet.",
        controller: "C(s) = 1",
        change: "This baseline shows why the uncompensated plant cannot meet the low-frequency tracking/disturbance requirements.",
        prompt: "Which specification is most obviously failing before any controller is added?",
      },
      {
        added: "Proportional gain",
        block: "C<sub>P</sub>(s) = 2",
        controller: "C(s) = 2",
        change: "The proportional gain shifts the magnitude curve up at all frequencies without adding phase.",
        prompt: "Does a pure gain solve the low-frequency requirements, or does it mainly move the whole loop shape?",
      },
      {
        added: "PI block",
        block: "C<sub>PI</sub>(s) = (s + 0.4) / s",
        controller: "C(s) = 2(s + 0.4) / s",
        change: "The PI block raises low-frequency gain and changes the system type, but it also adds phase lag near crossover.",
        prompt: "Which part of the plot changes most: low frequency, crossover, or high frequency?",
      },
      {
        added: "Lag block",
        block: "C<sub>Lag</sub>(s) = (s + 0.8) / (s + 0.8/50)",
        controller: "C(s) = 2(s + 0.4)(s + 0.8) / (s(s + 0.8/50))",
        change: "The lag block gives another low-frequency lift while trying to disturb crossover less than a large gain increase would.",
        prompt: "After adding lag, which remaining requirement points to the next compensator?",
      },
      {
        added: "Low-pass filter",
        block: "C<sub>LPF</sub>(s) = 5 / (s + 5)",
        controller: "C(s) = 2(s + 0.4)(s + 0.8)5 / (s(s + 0.8/50)(s + 5))",
        change: "The low-pass filter rolls off the high-frequency loop gain so the noise attenuation requirement can be satisfied.",
        prompt: "What tradeoff does the low-pass filter introduce in the phase plot?",
      },
    ];
    const ch18ControllerResponse = (step, w) => {
      const pMag = -10 * Math.log10(1 + w * w);
      let mag = pMag;
      let phase = -Math.atan(w) * 180 / Math.PI;
      if (step >= 1) mag += 20 * Math.log10(2);
      if (step >= 2) {
        const pi = ch18BlockResponse("pi", 0.4, 1, w);
        mag += pi.magDb;
        phase += pi.phase;
      }
      if (step >= 3) {
        const lag = ch18BlockResponse("lag", 0.8, 50, w);
        mag += lag.magDb;
        phase += lag.phase;
      }
      if (step >= 4) {
        const lpf = ch18BlockResponse("lpf", 5, 1, w);
        mag += lpf.magDb;
        phase += lpf.phase;
      }
      return { magDb: mag, phase };
    };
    const ch18FindGainCrossover = (step) => {
      const w = logspace(-4, 3, 720);
      let previousW = w[0];
      let previousMag = ch18ControllerResponse(step, previousW).magDb;
      for (const currentW of w.slice(1)) {
        const currentMag = ch18ControllerResponse(step, currentW).magDb;
        if ((previousMag <= 0 && currentMag >= 0) || (previousMag >= 0 && currentMag <= 0)) {
          let lo = previousW;
          let hi = currentW;
          for (let i = 0; i < 28; i += 1) {
            const mid = Math.sqrt(lo * hi);
            const loMag = ch18ControllerResponse(step, lo).magDb;
            const midMag = ch18ControllerResponse(step, mid).magDb;
            if ((loMag <= 0 && midMag <= 0) || (loMag >= 0 && midMag >= 0)) lo = mid;
            else hi = mid;
          }
          const wc = Math.sqrt(lo * hi);
          const phase = ch18ControllerResponse(step, wc).phase;
          return { wc, phase, phaseMargin: 180 + phase };
        }
        previousW = currentW;
        previousMag = currentMag;
      }
      return { wc: NaN, phase: NaN, phaseMargin: NaN };
    };
    const ch18DashboardResponse = (params, w) => {
      const plantMag = -10 * Math.log10(1 + w * w);
      const plantPhase = -Math.atan(w) * 180 / Math.PI;
      const pi = ch18BlockResponse("pi", params.piZero, 1, w);
      const lag = ch18BlockResponse("lag", params.lagZero, params.lagRatio, w);
      const lead = ch18BlockResponse("lead", params.leadCenter, params.leadRatio, w);
      const lpf = ch18BlockResponse("lpf", params.lpfPole, 1, w);
      return {
        plantMag,
        plantPhase,
        magDb: plantMag + params.gainDb + pi.magDb + lag.magDb + lead.magDb + lpf.magDb,
        phase: plantPhase + pi.phase + lag.phase + lead.phase + lpf.phase,
      };
    };
    const ch18FindDashboardCrossover = (params) => {
      const w = logspace(-4, 4, 900);
      let previousW = w[0];
      let previousMag = ch18DashboardResponse(params, previousW).magDb;
      for (const currentW of w.slice(1)) {
        const currentMag = ch18DashboardResponse(params, currentW).magDb;
        if ((previousMag <= 0 && currentMag >= 0) || (previousMag >= 0 && currentMag <= 0)) {
          let lo = previousW;
          let hi = currentW;
          for (let i = 0; i < 30; i += 1) {
            const mid = Math.sqrt(lo * hi);
            const loMag = ch18DashboardResponse(params, lo).magDb;
            const midMag = ch18DashboardResponse(params, mid).magDb;
            if ((loMag <= 0 && midMag <= 0) || (loMag >= 0 && midMag >= 0)) lo = mid;
            else hi = mid;
          }
          const wc = Math.sqrt(lo * hi);
          const phase = ch18DashboardResponse(params, wc).phase;
          return { wc, phase, phaseMargin: 180 + phase };
        }
        previousW = currentW;
        previousMag = currentMag;
      }
      return { wc: NaN, phase: NaN, phaseMargin: NaN };
    };
    const drawCh18CompensatorDashboard = (activity, params, metrics) => {
      const canvas = activity.querySelector("[data-plot='ch18-compensator-dashboard']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      const magPlot = { left: 74, right: width - 34, top: 52, bottom: 354 };
      const phasePlot = { left: 74, right: width - 34, top: 442, bottom: height - 62 };
      const xMin = -4;
      const xMax = 4;
      const yMin = -100;
      const yMax = 130;
      const phaseMin = -260;
      const phaseMax = 40;
      const xMap = (omega) => magPlot.left + ((Math.log10(omega) - xMin) / (xMax - xMin)) * (magPlot.right - magPlot.left);
      const magY = (db) => magPlot.top + ((yMax - db) / (yMax - yMin)) * (magPlot.bottom - magPlot.top);
      const phaseY = (deg) => phasePlot.top + ((phaseMax - deg) / (phaseMax - phaseMin)) * (phasePlot.bottom - phasePlot.top);
      const w = logspace(xMin, xMax, 680);
      const response = w.map((x) => ({ x, ...ch18DashboardResponse(params, x) }));
      const path = (points, yAccessor, yMapper) => points.map((p, i) => `${i === 0 ? "M" : "L"}${xMap(p.x).toFixed(1)} ${yMapper(yAccessor(p)).toFixed(1)}`).join(" ");
      const drawPath = (points, yAccessor, yMapper, color, widthPx = 2.5, dash = [], clipPlot = null) => {
        ctx.save();
        if (clipPlot) {
          ctx.beginPath();
          ctx.rect(clipPlot.left, clipPlot.top, clipPlot.right - clipPlot.left, clipPlot.bottom - clipPlot.top);
          ctx.clip();
        }
        ctx.strokeStyle = color;
        ctx.lineWidth = widthPx;
        ctx.setLineDash(dash);
        ctx.beginPath();
        ctx.stroke(new Path2D(path(points, yAccessor, yMapper)));
        ctx.restore();
      };
      const drawFrame = (plot, yTicks, yMapper, label) => {
        ctx.strokeStyle = "#dbe7ea";
        ctx.lineWidth = 1;
        ctx.fillStyle = "#526047";
        ctx.font = "12px sans-serif";
        for (let exp = xMin; exp <= xMax; exp += 1) {
          const x = xMap(10 ** exp);
          ctx.beginPath();
          ctx.moveTo(x, plot.top);
          ctx.lineTo(x, plot.bottom);
          ctx.stroke();
          if (plot === phasePlot) ctx.fillText(`10^${exp}`, x - 16, height - 36);
        }
        for (const tick of yTicks) {
          const y = yMapper(tick);
          ctx.beginPath();
          ctx.moveTo(plot.left, y);
          ctx.lineTo(plot.right, y);
          ctx.stroke();
          ctx.fillText(String(tick), 28, y + 4);
        }
        ctx.strokeStyle = "#172d33";
        ctx.strokeRect(plot.left, plot.top, plot.right - plot.left, plot.bottom - plot.top);
        ctx.fillStyle = "#172d33";
        ctx.font = "700 14px sans-serif";
        ctx.fillText(label, plot.left, plot.top - 16);
      };

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);
      drawFrame(magPlot, [-80, -40, 0, 40, 80, 120], magY, "Magnitude");
      drawFrame(phasePlot, [-240, -180, -120, -60, 0], phaseY, "Phase");

      ctx.save();
      ctx.fillStyle = "rgba(111, 127, 50, 0.12)";
      ctx.fillRect(xMap(1e-4), magPlot.top, xMap(0.03) - xMap(1e-4), magPlot.bottom - magPlot.top);
      ctx.fillStyle = "rgba(156, 71, 53, 0.1)";
      ctx.fillRect(xMap(100), magPlot.top, xMap(1e4) - xMap(100), magPlot.bottom - magPlot.top);
      ctx.strokeStyle = "#3a8f44";
      ctx.lineWidth = 2;
      ctx.setLineDash([8, 5]);
      ctx.beginPath();
      ctx.moveTo(xMap(1e-4), magY(30));
      ctx.lineTo(xMap(0.03), magY(30));
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(xMap(100), magY(-20));
      ctx.lineTo(xMap(1e4), magY(-20));
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = "#526047";
      ctx.font = "12px sans-serif";
      ctx.fillText("tracking: |PC| >= 30 dB", xMap(1.2e-4), magY(30) - 8);
      ctx.fillText("noise: |PC| <= -20 dB", xMap(105), magY(-20) - 8);
      ctx.restore();

      drawPath(response, (p) => p.plantMag, magY, "#7f8d93", 2.3, [7, 5], magPlot);
      drawPath(response, (p) => p.magDb, magY, "#0e6d77", 3, [], magPlot);
      drawPath(response, (p) => p.phase, phaseY, "#d56b35", 3, [], phasePlot);

      ctx.save();
      ctx.strokeStyle = "#7f8d93";
      ctx.lineWidth = 1.6;
      ctx.setLineDash([7, 5]);
      ctx.beginPath();
      ctx.moveTo(magPlot.left, magY(0));
      ctx.lineTo(magPlot.right, magY(0));
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(phasePlot.left, phaseY(-180));
      ctx.lineTo(phasePlot.right, phaseY(-180));
      ctx.stroke();
      ctx.restore();

      if (Number.isFinite(metrics.margin.wc)) {
        const x = xMap(metrics.margin.wc);
        ctx.save();
        ctx.strokeStyle = "#8b5e24";
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 5]);
        ctx.beginPath();
        ctx.moveTo(x, magPlot.top);
        ctx.lineTo(x, phasePlot.bottom);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = "#8b5e24";
        ctx.font = "12px sans-serif";
        ctx.fillText(`wc = ${fmt(metrics.margin.wc, 2)} rad/s`, Math.min(x + 8, magPlot.right - 122), magPlot.top + 20);
        ctx.beginPath();
        ctx.arc(x, phaseY(metrics.margin.phase), 5, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillText(`PM = ${fmt(metrics.margin.phaseMargin, 1)} deg`, Math.min(x + 8, phasePlot.right - 112), phaseY(metrics.margin.phase) - 10);
        ctx.restore();
      }

      const status = (x, y, ok, label) => {
        ctx.fillStyle = ok ? "#2f7d46" : "#b64b35";
        ctx.beginPath();
        ctx.arc(x, y, 7, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "#172d33";
        ctx.font = "12px sans-serif";
        ctx.fillText(label, x + 13, y + 4);
      };
      ctx.fillStyle = "#172d33";
      ctx.font = "13px sans-serif";
      ctx.fillText("gray dashed: P(s); blue: P(s)C(s); orange: phase of P(s)C(s); green guides: requirements", magPlot.left + 4, 28);
      status(magPlot.left + 4, 384, metrics.lowOk, "low-frequency gain");
      status(magPlot.left + 188, 384, metrics.marginOk, "phase margin");
      status(magPlot.left + 340, 384, metrics.bandwidthOk, "bandwidth");
      status(magPlot.left + 482, 384, metrics.noiseOk, "noise attenuation");
      ctx.fillStyle = "#526047";
      ctx.fillText("frequency (rad/s)", phasePlot.right - 120, height - 18);
    };
    const drawCh18LoopshapeSequence = (activity, step) => {
      const canvas = activity.querySelector("[data-plot='ch18-loopshape-sequence']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      const magPlot = { left: 74, right: width - 32, top: 46, bottom: 342 };
      const phasePlot = { left: 74, right: width - 32, top: 426, bottom: height - 58 };
      const xMin = -4;
      const xMax = 3;
      const yMin = -80;
      const yMax = 110;
      const phaseMin = -220;
      const phaseMax = 20;
      const xMap = (omega) => magPlot.left + ((Math.log10(omega) - xMin) / (xMax - xMin)) * (magPlot.right - magPlot.left);
      const magY = (db) => magPlot.top + ((yMax - db) / (yMax - yMin)) * (magPlot.bottom - magPlot.top);
      const phaseY = (deg) => phasePlot.top + ((phaseMax - deg) / (phaseMax - phaseMin)) * (phasePlot.bottom - phasePlot.top);
      const w = logspace(xMin, xMax, 520);
      const plant = w.map((x) => ({ x, y: -10 * Math.log10(1 + x * x) }));
      const current = w.map((x) => ({ x, ...ch18ControllerResponse(step, x) }));
      const previous = step > 0 ? w.map((x) => ({ x, ...ch18ControllerResponse(step - 1, x) })) : null;
      const margin = ch18FindGainCrossover(step);
      const path = (points, yAccessor, yMapper) => points.map((p, i) => `${i === 0 ? "M" : "L"}${xMap(p.x).toFixed(1)} ${yMapper(yAccessor(p)).toFixed(1)}`).join(" ");
      const drawPath = (points, yAccessor, yMapper, color, widthPx = 2.5, dash = [], clipPlot = null) => {
        ctx.save();
        if (clipPlot) {
          ctx.beginPath();
          ctx.rect(clipPlot.left, clipPlot.top, clipPlot.right - clipPlot.left, clipPlot.bottom - clipPlot.top);
          ctx.clip();
        }
        ctx.strokeStyle = color;
        ctx.lineWidth = widthPx;
        ctx.setLineDash(dash);
        ctx.beginPath();
        ctx.stroke(new Path2D(path(points, yAccessor, yMapper)));
        ctx.restore();
      };

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);
      ctx.font = "12px sans-serif";

      const drawGrid = (plot, yTicks, yMapper, label) => {
        ctx.strokeStyle = "#dbe7ea";
        ctx.lineWidth = 1;
        ctx.fillStyle = "#526047";
        for (let i = xMin; i <= xMax; i += 1) {
          const x = xMap(10 ** i);
          ctx.beginPath();
          ctx.moveTo(x, plot.top);
          ctx.lineTo(x, plot.bottom);
          ctx.stroke();
          if (plot === phasePlot) ctx.fillText(`10^${i}`, x - 16, height - 34);
        }
        for (const tick of yTicks) {
          const y = yMapper(tick);
          ctx.beginPath();
          ctx.moveTo(plot.left, y);
          ctx.lineTo(plot.right, y);
          ctx.stroke();
          ctx.fillText(String(tick), 28, y + 4);
        }
        ctx.strokeStyle = "#172d33";
        ctx.strokeRect(plot.left, plot.top, plot.right - plot.left, plot.bottom - plot.top);
        ctx.fillStyle = "#172d33";
        ctx.font = "700 14px sans-serif";
        ctx.fillText(label, plot.left, plot.top - 14);
        ctx.font = "12px sans-serif";
      };

      drawGrid(magPlot, [-60, -40, -20, 0, 20, 40, 60, 80, 100], magY, "Magnitude");
      drawGrid(phasePlot, [-180, -135, -90, -45, 0], phaseY, "Phase");

      ctx.save();
      ctx.strokeStyle = "#3a8f44";
      ctx.lineWidth = 2;
      ctx.setLineDash([8, 5]);
      ctx.beginPath();
      ctx.moveTo(xMap(1e-4), magY(20 * Math.log10(1 / 0.03) - 20 * Math.log10(1e-4)));
      ctx.lineTo(xMap(1), magY(20 * Math.log10(1 / 0.03)));
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(xMap(1e-4), magY(20));
      ctx.lineTo(xMap(0.1), magY(20));
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(xMap(10), magY(-20));
      ctx.lineTo(xMap(1000), magY(-20));
      ctx.stroke();
      ctx.restore();

      drawPath(plant, (p) => p.y, magY, "#7f8d93", 2, [], magPlot);
      if (previous) {
        drawPath(previous, (p) => p.magDb, magY, "#0e6d77", 2.4, [8, 6], magPlot);
        drawPath(previous, (p) => p.phase, phaseY, "#0e6d77", 2.2, [8, 6], phasePlot);
      }
      drawPath(current, (p) => p.magDb, magY, "#0e6d77", 3, [], magPlot);
      drawPath(current, (p) => p.phase, phaseY, "#d56b35", 3, [], phasePlot);

      if (Number.isFinite(margin.wc)) {
        const x = xMap(margin.wc);
        ctx.save();
        ctx.strokeStyle = "#8b5e24";
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 5]);
        ctx.beginPath();
        ctx.moveTo(x, magPlot.top);
        ctx.lineTo(x, phasePlot.bottom);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = "#8b5e24";
        ctx.font = "12px sans-serif";
        ctx.fillText(`wc = ${fmt(margin.wc, 2)} rad/s`, Math.min(x + 8, magPlot.right - 116), magPlot.top + 18);
        ctx.beginPath();
        ctx.arc(x, phaseY(margin.phase), 5, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
      }

      ctx.fillStyle = "#172d33";
      ctx.font = "13px sans-serif";
      ctx.fillText("gray: P(s); dashed blue: previous PC; solid blue: current PC; orange: current phase; green: design specs", magPlot.left + 10, 24);
      ctx.fillText("frequency (rad/s)", width - 140, height - 16);
    };
    const drawCh18Prefilter = (activity, p) => {
      const canvas = activity.querySelector("[data-plot='ch18-prefilter']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      const top = { left: 64, right: width - 28, top: 34, bottom: 230 };
      const bottom = { left: 64, right: width - 28, top: 306, bottom: height - 46 };
      const wMinExp = -1;
      const wMaxExp = 3;
      const w = logspace(wMinExp, wMaxExp, 460);
      const clMag = (omega) => {
        const r = omega / 9;
        return 20 * Math.log10(1.25 / Math.hypot(1 - r * r, 0.7 * r));
      };
      const fMag = (omega) => -10 * Math.log10(1 + (omega / p) ** 2);
      const xTop = (omega) => top.left + ((Math.log10(omega) - wMinExp) / (wMaxExp - wMinExp)) * (top.right - top.left);
      const yTop = (db) => top.bottom - ((db + 62) / 80) * (top.bottom - top.top);
      const xBot = (t) => bottom.left + (t / 8) * (bottom.right - bottom.left);
      const yBot = (y) => bottom.bottom - ((y - 0) / 1.6) * (bottom.bottom - bottom.top);
      const path = (points, xFn, yFn) => points.map((pt, i) => `${i === 0 ? "M" : "L"}${xFn(pt.x).toFixed(1)} ${yFn(pt.y).toFixed(1)}`).join(" ");
      const legend = (items, x, y) => {
        ctx.save();
        ctx.font = "11px sans-serif";
        ctx.textBaseline = "middle";
        const widths = items.map((item) => 34 + ctx.measureText(item.label).width);
        const boxWidth = Math.max(...widths) + 18;
        const boxHeight = 10 + items.length * 18;
        ctx.fillStyle = "rgba(255, 255, 255, 0.88)";
        ctx.strokeStyle = "#cbdadd";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.roundRect?.(x, y, boxWidth, boxHeight, 7);
        if (!ctx.roundRect) ctx.rect(x, y, boxWidth, boxHeight);
        ctx.fill();
        ctx.stroke();
        items.forEach((item, index) => {
          const yLine = y + 12 + index * 18;
          ctx.strokeStyle = item.color;
          ctx.lineWidth = 2.4;
          ctx.setLineDash(item.dash || []);
          ctx.beginPath();
          ctx.moveTo(x + 9, yLine);
          ctx.lineTo(x + 32, yLine);
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.fillStyle = "#172d33";
          ctx.fillText(item.label, x + 39, yLine);
        });
        ctx.restore();
      };
      const baseStep = Array.from({ length: 300 }, (_, i) => {
        const t = (8 * i) / 299;
        return { x: t, y: 1 - Math.exp(-0.42 * t) * (Math.cos(2.15 * t) + 0.22 * Math.sin(2.15 * t)) };
      });
      const filteredStep = Array.from({ length: 300 }, (_, i) => {
        const t = (8 * i) / 299;
        const ringScale = Math.max(0.12, Math.min(0.9, p / 18));
        return { x: t, y: 1 - Math.exp(-0.34 * Math.min(p / 9, 1.6) * t) * (Math.cos(1.55 * t) * ringScale + 0.18 * Math.sin(1.55 * t)) };
      });
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);
      for (const plot of [top, bottom]) {
        ctx.strokeStyle = "#dbe7ea";
        ctx.strokeRect(plot.left, plot.top, plot.right - plot.left, plot.bottom - plot.top);
      }
      ctx.save();
      ctx.beginPath();
      ctx.rect(top.left, top.top, top.right - top.left, top.bottom - top.top);
      ctx.clip();
      ctx.strokeStyle = "#7f8d93";
      ctx.lineWidth = 2.2;
      ctx.stroke(new Path2D(path(w.map((x) => ({ x, y: clMag(x) })), xTop, yTop)));
      ctx.strokeStyle = "#d56b35";
      ctx.stroke(new Path2D(path(w.map((x) => ({ x, y: clMag(x) + fMag(x) })), xTop, yTop)));
      ctx.strokeStyle = "#0e6d77";
      ctx.stroke(new Path2D(path(w.map((x) => ({ x, y: fMag(x) })), xTop, yTop)));
      ctx.restore();
      ctx.strokeStyle = "#7f8d93";
      ctx.setLineDash([7, 5]);
      ctx.stroke(new Path2D(path(baseStep, xBot, yBot)));
      ctx.setLineDash([]);
      ctx.strokeStyle = "#d56b35";
      ctx.stroke(new Path2D(path(filteredStep, xBot, yBot)));
      ctx.fillStyle = "#172d33";
      ctx.font = "13px sans-serif";
      ctx.fillText("Bode magnitude: closed loop, prefilter, and filtered reference path", top.left, 24);
      ctx.fillText("Step response: dashed without prefilter, solid with prefilter", bottom.left, bottom.top - 12);
      legend([
        { label: "closed loop T", color: "#7f8d93" },
        { label: "prefilter F", color: "#0e6d77" },
        { label: "prefiltered path FT", color: "#d56b35" },
      ], top.right - 174, top.top + 12);
      legend([
        { label: "without prefilter", color: "#7f8d93", dash: [7, 5] },
        { label: "with prefilter", color: "#d56b35" },
      ], bottom.right - 174, bottom.top + 12);
      ctx.fillStyle = "#526047";
      ctx.font = "11px sans-serif";
      for (let exp = wMinExp; exp <= wMaxExp; exp += 1) {
        const x = xTop(10 ** exp);
        ctx.fillText(`10^${exp}`, x - 12, top.bottom + 16);
      }
      ctx.fillText("frequency (rad/s)", top.right - 106, top.bottom + 32);
    };
    const frequencySpecPlants = {
      constant: {
        label: "P(s) = 1",
        labelHtml: "<span class=\"inline-equation\">P(s) = 1</span>",
        magDb: () => 0,
      },
      integrator: {
        label: "P(s) = 1/s",
        labelHtml: "<span class=\"inline-equation\">P(s) = 1/s</span>",
        magDb: (omega) => -20 * Math.log10(omega),
      },
      "double-integrator": {
        label: "P(s) = 1/s^2",
        labelHtml: "<span class=\"inline-equation\">P(s) = 1/s<sup>2</sup></span>",
        magDb: (omega) => -40 * Math.log10(omega),
      },
      "first-order": {
        label: "P(s) = 1/(s+1)",
        labelHtml: "<span class=\"inline-equation\">P(s) = 1/(s+1)</span>",
        magDb: (omega) => -10 * Math.log10(1 + omega * omega),
      },
      "zero-integrator": {
        label: "P(s) = (s+1)/(s(s+5))",
        labelHtml: "<span class=\"inline-equation\">P(s) = (s+1)/(s(s+5))</span>",
        magDb: (omega) => (
          10 * Math.log10(1 + omega * omega)
          - 20 * Math.log10(omega)
          - 10 * Math.log10(25 + omega * omega)
        ),
      },
    };

    const frequencySpecDefinitions = {
      tracking: {
        title: "Reference tracking",
        frequencyLabel: "&omega;<sub>r</sub>",
        region: "low",
        accepts: "above",
        defaultLevelDb: 20,
        guide: () => null,
        measurement: "B<sub>r</sub> = &vert;PC&vert;<sub>dB</sub>",
        boundLabel: "&gamma;<sub>r</sub>",
        note: (gamma) => `For reference content below <span class="inline-equation">&omega;<sub>r</sub></span>, the approximate error bound is <span class="inline-equation">&vert;e&vert; &le; ${fmt(gamma, 3)}&vert;r&vert;</span> when <span class="inline-equation">&vert;PC&vert;</span> is large.`,
      },
      noise: {
        title: "Noise attenuation",
        frequencyLabel: "&omega;<sub>no</sub>",
        region: "high",
        accepts: "below",
        defaultLevelDb: -20,
        guide: () => null,
        measurement: "B<sub>n</sub> from &vert;PC&vert;<sub>dB</sub>",
        boundLabel: "&gamma;<sub>n</sub>",
        note: (gamma) => `For noise content above <span class="inline-equation">&omega;<sub>no</sub></span>, the approximate error bound is <span class="inline-equation">&vert;e&vert; &le; ${fmt(gamma, 3)}&vert;n&vert;</span> when <span class="inline-equation">&vert;PC&vert;</span> is small.`,
      },
      "output-disturbance": {
        title: "Output disturbance rejection",
        frequencyLabel: "&omega;<sub>d,out</sub>",
        region: "low",
        accepts: "above",
        defaultLevelDb: 20,
        guide: () => null,
        measurement: "B<sub>d,out</sub> = &vert;PC&vert;<sub>dB</sub>",
        boundLabel: "&gamma;<sub>d,out</sub>",
        note: (gamma) => `For output disturbances below <span class="inline-equation">&omega;<sub>d,out</sub></span>, the approximate bound is <span class="inline-equation">&vert;e&vert; &le; ${fmt(gamma, 3)}&vert;d<sub>out</sub>&vert;</span> when <span class="inline-equation">&vert;PC&vert;</span> is large.`,
      },
      "input-disturbance": {
        title: "Input disturbance rejection",
        frequencyLabel: "&omega;<sub>d,in</sub>",
        region: "low",
        accepts: "above",
        defaultLevelDb: 20,
        guide: (omega, plantDb) => plantDb,
        measurement: "B<sub>d,in</sub> = &vert;PC&vert;<sub>dB</sub> - &vert;P&vert;<sub>dB</sub>",
        boundLabel: "&gamma;<sub>d,in</sub>",
        note: (gamma) => `For input disturbances below <span class="inline-equation">&omega;<sub>d,in</sub></span>, the loop gain must exceed the plant magnitude; this gives <span class="inline-equation">&vert;e&vert; &le; ${fmt(gamma, 3)}&vert;d<sub>in</sub>&vert;</span>.`,
      },
      type0: {
        title: "Type 0 step tracking",
        frequencyLabel: "low-frequency &omega;",
        region: "low",
        accepts: "above",
        defaultLevelDb: 20,
        guide: () => 0,
        measurement: "B<sub>0</sub> = &vert;PC&vert;<sub>dB</sub>",
        boundLabel: "step steady-state error",
        note: (error, m) => `A type 0 system has finite step error: <span class="inline-equation">e<sub>ss</sub> = A/(1 + M<sub>p</sub>)</span>, with <span class="inline-equation">M<sub>p</sub> = ${fmt(m, 3)}</span>.`,
      },
      type1: {
        title: "Type 1 ramp tracking",
        frequencyLabel: "low-frequency &omega;",
        region: "low",
        accepts: "above",
        defaultLevelDb: 20,
        guide: (omega) => -20 * Math.log10(omega),
        measurement: "B<sub>1</sub> = &vert;PC&vert;<sub>dB</sub> - &vert;1/(j&omega;)&vert;<sub>dB</sub>",
        boundLabel: "ramp steady-state error",
        note: (error, m) => `A type 1 system has finite ramp error: <span class="inline-equation">e<sub>ss</sub> = A/M<sub>v</sub></span>, with <span class="inline-equation">M<sub>v</sub> = ${fmt(m, 3)}</span>.`,
      },
      type2: {
        title: "Type 2 parabola tracking",
        frequencyLabel: "low-frequency &omega;",
        region: "low",
        accepts: "above",
        defaultLevelDb: 20,
        guide: (omega) => -40 * Math.log10(omega),
        measurement: "B<sub>2</sub> = &vert;PC&vert;<sub>dB</sub> - &vert;1/(j&omega;)<sup>2</sup>&vert;<sub>dB</sub>",
        boundLabel: "parabola steady-state error",
        note: (error, m) => `A type 2 system has finite parabolic-input error: <span class="inline-equation">e<sub>ss</sub> = A/M<sub>a</sub></span>, with <span class="inline-equation">M<sub>a</sub> = ${fmt(m, 3)}</span>.`,
      },
    };

    const drawFrequencySpecCanvas = (activity, values) => {
      const canvas = activity.querySelector("[data-plot='frequency-spec']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;

      const width = canvas.width;
      const height = canvas.height;
      const pad = { left: 74, right: 30, top: 34, bottom: 62 };
      const xMin = -2;
      const xMax = 2;
      const yMin = -80;
      const yMax = 80;
      const plotW = width - pad.left - pad.right;
      const plotH = height - pad.top - pad.bottom;
      const xMap = (logOmega) => pad.left + ((logOmega - xMin) / (xMax - xMin)) * plotW;
      const yMap = (db) => pad.top + ((yMax - db) / (yMax - yMin)) * plotH;
      const clampDb = (db) => Math.max(yMin, Math.min(yMax, db));
      const plant = values.plant;
      const spec = values.spec;
      const thresholdDb = (omega) => {
        const plantDb = plant.magDb(omega);
        const guideDb = spec.guide(omega, plantDb);
        const baseDb = guideDb !== null && Number.isFinite(guideDb) ? guideDb : 0;
        return baseDb + values.specDb;
      };

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);

      const selectedLog = Math.log10(values.omega);
      const selectedX = xMap(selectedLog);
      const selectedThresholdDb = thresholdDb(values.omega);
      const selectedThresholdY = yMap(clampDb(selectedThresholdDb));
      const regionStart = spec.region === "high" ? Math.max(xMin, selectedLog) : xMin;
      const regionEnd = spec.region === "high" ? xMax : Math.min(xMax, selectedLog);
      if (regionEnd > regionStart) {
        const samples = 120;
        ctx.save();
        ctx.beginPath();
        ctx.rect(pad.left, pad.top, plotW, plotH);
        ctx.clip();
        ctx.fillStyle = "rgba(102, 124, 43, 0.15)";
        ctx.beginPath();
        if (spec.accepts === "below") {
          ctx.moveTo(xMap(regionStart), pad.top + plotH);
          ctx.lineTo(xMap(regionEnd), pad.top + plotH);
          for (let i = samples; i >= 0; i -= 1) {
            const logW = regionStart + (i / samples) * (regionEnd - regionStart);
            const omega = 10 ** logW;
            ctx.lineTo(xMap(logW), yMap(clampDb(thresholdDb(omega))));
          }
        } else {
          ctx.moveTo(xMap(regionStart), pad.top);
          ctx.lineTo(xMap(regionEnd), pad.top);
          for (let i = samples; i >= 0; i -= 1) {
            const logW = regionStart + (i / samples) * (regionEnd - regionStart);
            const omega = 10 ** logW;
            ctx.lineTo(xMap(logW), yMap(clampDb(thresholdDb(omega))));
          }
        }
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }

      ctx.strokeStyle = "#dce8eb";
      ctx.lineWidth = 1;
      ctx.font = "12px sans-serif";
      ctx.fillStyle = "#526047";
      for (let logW = xMin; logW <= xMax; logW += 1) {
        const x = xMap(logW);
        ctx.beginPath();
        ctx.moveTo(x, pad.top);
        ctx.lineTo(x, pad.top + plotH);
        ctx.stroke();
        ctx.fillText(`10^${logW}`, x - 14, height - 30);
      }
      for (let db = yMin; db <= yMax; db += 20) {
        const y = yMap(db);
        ctx.beginPath();
        ctx.moveTo(pad.left, y);
        ctx.lineTo(pad.left + plotW, y);
        ctx.stroke();
        ctx.fillText(String(db), 28, y + 4);
      }

      ctx.strokeStyle = "#172d33";
      ctx.lineWidth = 1.8;
      ctx.strokeRect(pad.left, pad.top, plotW, plotH);
      ctx.fillStyle = "#172d33";
      ctx.font = "14px sans-serif";
      ctx.fillText("dB", 24, pad.top + 4);
      ctx.fillText("ω (rad/sec)", width - 118, height - 16);

      const drawCurve = (fn, color, lineWidth, dash = []) => {
        ctx.save();
        ctx.beginPath();
        ctx.rect(pad.left, pad.top, plotW, plotH);
        ctx.clip();
        ctx.beginPath();
        ctx.setLineDash(dash);
        for (let i = 0; i <= 260; i += 1) {
          const logW = xMin + (i / 260) * (xMax - xMin);
          const omega = 10 ** logW;
          const y = yMap(fn(omega));
          const x = xMap(logW);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        }
        ctx.strokeStyle = color;
        ctx.lineWidth = lineWidth;
        ctx.stroke();
        ctx.restore();
      };

      drawCurve((omega) => plant.magDb(omega), "#7b8790", 2, [7, 5]);
      drawCurve((omega) => plant.magDb(omega) + values.offsetDb, "#0e6d77", 3);
      drawCurve((omega) => thresholdDb(omega), "#667c2b", 2.6, [6, 4]);
      if (values.guideDb !== null) drawCurve((omega) => spec.guide(omega, plant.magDb(omega)), "#d56b35", 1.8, [3, 5]);

      const selectedY = yMap(clampDb(values.loopDb));
      const guideY = yMap(clampDb(values.boundaryDb));
      ctx.strokeStyle = "#172d33";
      ctx.lineWidth = 1.6;
      ctx.setLineDash([7, 5]);
      ctx.beginPath();
      ctx.moveTo(selectedX, pad.top);
      ctx.lineTo(selectedX, pad.top + plotH);
      ctx.moveTo(pad.left, selectedY);
      ctx.lineTo(pad.left + plotW, selectedY);
      ctx.moveTo(pad.left, selectedThresholdY);
      ctx.lineTo(pad.left + plotW, selectedThresholdY);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.strokeStyle = values.meetsSpec ? "#0e6d77" : "#9c4735";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(selectedX, selectedY);
      ctx.lineTo(selectedX, guideY);
      ctx.stroke();

      ctx.fillStyle = "#0e6d77";
      ctx.beginPath();
      ctx.arc(selectedX, selectedY, 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = "#667c2b";
      ctx.beginPath();
      ctx.arc(selectedX, selectedThresholdY, 5, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = "rgba(255, 255, 255, 0.92)";
      ctx.strokeStyle = "#dbe7ea";
      ctx.lineWidth = 1;
      ctx.fillRect(width - 350, 44, 312, 124);
      ctx.strokeRect(width - 350, 44, 312, 124);
      ctx.font = "13px sans-serif";
      ctx.fillStyle = "#667c2b";
      ctx.fillText("green shade: acceptable region", width - 330, 68);
      ctx.fillStyle = "#7b8790";
      ctx.fillText("gray dashed: |P(jω)|", width - 330, 92);
      ctx.fillStyle = "#0e6d77";
      ctx.fillText("blue: |P(jω)C(jω)|", width - 330, 116);
      ctx.fillStyle = "#667c2b";
      ctx.fillText("green dashed: spec boundary", width - 330, 140);
      ctx.fillStyle = "#172d33";
      ctx.fillText(`${spec.title}: measured ${fmt(values.metricDb, 1)} dB`, width - 330, 160);
    };

    const transferFractionHtml = (top, bottom) => `<span class="inline-frac"><span class="frac-top">${top}</span><span class="frac-bottom">${bottom}</span></span>`;
    const signalTransferHtml = (inputHtml, outputTopHtml, outputBottomHtml) => (
      `<span class="metric-equation">${transferFractionHtml("E(s)", inputHtml)} = ${transferFractionHtml(outputTopHtml, outputBottomHtml)}</span>`
    );
    const signalPathDefinitions = {
      reference: {
        title: "Reference input R(s)",
        transfer: signalTransferHtml("R(s)", "1", "1 + P(s)C(s)"),
        note: "Reference tracking uses the sensitivity-like path to error, so large low-frequency loop gain makes tracking error small.",
      },
      noise: {
        title: "Sensor noise N(s)",
        transfer: signalTransferHtml("N(s)", "P(s)C(s)", "1 + P(s)C(s)"),
        note: "Noise enters the feedback measurement path. High-frequency noise attenuation requires small loop gain so PC/(1+PC) is small.",
      },
      "output-disturbance": {
        title: "Output disturbance D_out(s)",
        transfer: signalTransferHtml("D<sub>out</sub>(s)", "1", "1 + P(s)C(s)"),
        note: "Output disturbances use the same denominator and sensitivity shape as reference tracking, so large low-frequency loop gain rejects them.",
      },
      "input-disturbance": {
        title: "Input disturbance D_in(s)",
        transfer: signalTransferHtml("D<sub>in</sub>(s)", "P(s)", "1 + P(s)C(s)"),
        note: "Input disturbances pass through the plant before the loop suppresses them, so the design compares loop gain |PC| against plant magnitude |P|.",
      },
    };

    const drawSignalPathCanvas = (activity, sourceKey) => {
      const canvas = activity.querySelector("[data-plot='signal-path']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      const active = "#d56b35";
      const ink = "#172d33";
      const muted = "#9fb3b9";
      const fill = "#fbfdfe";
      const blockFill = "#ffffff";
      const isActive = (...keys) => keys.includes(sourceKey);
      const color = (...keys) => isActive(...keys) ? active : ink;

      const arrow = (x1, y1, x2, y2, stroke = ink, lineWidth = 3) => {
        const angle = Math.atan2(y2 - y1, x2 - x1);
        const size = 10;
        ctx.save();
        ctx.strokeStyle = stroke;
        ctx.fillStyle = stroke;
        ctx.lineWidth = lineWidth;
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(x2, y2);
        ctx.lineTo(x2 - size * Math.cos(angle - Math.PI / 6), y2 - size * Math.sin(angle - Math.PI / 6));
        ctx.lineTo(x2 - size * Math.cos(angle + Math.PI / 6), y2 - size * Math.sin(angle + Math.PI / 6));
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      };
      const block = (x, y, w, h, label, stroke = ink) => {
        ctx.save();
        ctx.fillStyle = blockFill;
        ctx.strokeStyle = stroke;
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.roundRect?.(x, y, w, h, 8);
        if (!ctx.roundRect) ctx.rect(x, y, w, h);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = ink;
        ctx.font = "700 24px sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(label, x + w / 2, y + h / 2);
        ctx.restore();
      };
      const sum = (x, y, stroke = ink) => {
        ctx.save();
        ctx.strokeStyle = stroke;
        ctx.lineWidth = 3;
        ctx.fillStyle = "#fff";
        ctx.beginPath();
        ctx.arc(x, y, 30, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = ink;
        ctx.font = "700 18px sans-serif";
        ctx.textAlign = "center";
        ctx.fillText("+", x - 22, y - 30);
        ctx.fillText("-", x - 26, y + 40);
        ctx.restore();
      };

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = fill;
      ctx.fillRect(0, 0, width, height);
      ctx.font = "700 18px sans-serif";
      ctx.fillStyle = ink;
      ctx.fillText("General feedback loop: choose an input and trace its transfer to error E(s)", 42, 36);

      const s1 = { x: 170, y: 160 };
      const c = { x: 250, y: 125, w: 115, h: 70 };
      const s2 = { x: 445, y: 160 };
      const p = { x: 515, y: 125, w: 115, h: 70 };
      const outX = 735;
      const feedbackY = 300;

      arrow(44, s1.y, s1.x - 32, s1.y, color("reference"), isActive("reference") ? 5 : 3);
      sum(s1.x, s1.y, color("reference", "noise", "output-disturbance", "input-disturbance"));
      arrow(s1.x + 30, s1.y, c.x, s1.y, color("reference", "noise", "output-disturbance", "input-disturbance"), 4);
      block(c.x, c.y, c.w, c.h, "C(s)", color("reference", "noise", "output-disturbance", "input-disturbance"));
      arrow(c.x + c.w, s1.y, s2.x - 32, s2.y, color("reference", "noise", "output-disturbance", "input-disturbance"), 4);
      sum(s2.x, s2.y, color("input-disturbance"));
      arrow(s2.x + 30, s2.y, p.x, s2.y, color("reference", "noise", "output-disturbance", "input-disturbance"), 4);
      block(p.x, p.y, p.w, p.h, "P(s)", color("reference", "noise", "output-disturbance", "input-disturbance"));
      arrow(p.x + p.w, s2.y, outX, s2.y, color("reference", "noise", "output-disturbance", "input-disturbance"), 4);

      arrow(s2.x, 64, s2.x, s2.y - 32, color("input-disturbance"), isActive("input-disturbance") ? 5 : 3);
      arrow(outX - 20, 64, outX - 20, s2.y - 2, color("output-disturbance"), isActive("output-disturbance") ? 5 : 3);
      arrow(outX - 20, s2.y, outX - 20, feedbackY, color("reference", "noise", "output-disturbance", "input-disturbance"), 3);
      arrow(outX - 20, feedbackY, s1.x, feedbackY, color("reference", "noise", "output-disturbance", "input-disturbance"), 3);
      arrow(s1.x, feedbackY, s1.x, s1.y + 32, color("reference", "noise", "output-disturbance", "input-disturbance"), 3);
      arrow(outX + 42, feedbackY, outX - 22, feedbackY, color("noise"), isActive("noise") ? 5 : 3);

      ctx.fillStyle = color("reference");
      ctx.font = "700 24px serif";
      ctx.fillText("R", 28, s1.y + 7);
      ctx.fillStyle = color("input-disturbance");
      ctx.fillText("D_in", s2.x - 24, 54);
      ctx.fillStyle = color("output-disturbance");
      ctx.fillText("D_out", outX - 62, 54);
      ctx.fillStyle = color("noise");
      ctx.fillText("N", outX + 52, feedbackY + 7);
      ctx.fillStyle = ink;
      ctx.fillText("y", outX + 16, s2.y - 12);
      ctx.font = "700 18px sans-serif";
      ctx.fillStyle = active;
      ctx.fillText(signalPathDefinitions[sourceKey]?.title || signalPathDefinitions.reference.title, 42, 390);
      ctx.fillStyle = muted;
      ctx.fillText("The denominator 1 + P(s)C(s) appears because the signal circulates around the feedback loop.", 42, 414);
    };

    const systemTypeExamples = {
      "unknown-a": {
        label: "Unknown loop A",
        type: 0,
        constantDb: 22,
        corner: 0.9,
        note: "This loop has a flat low-frequency magnitude, so it behaves like a Type 0 loop.",
      },
      "unknown-b": {
        label: "Unknown loop B",
        type: 1,
        constantDb: 14,
        corner: 0.7,
        note: "This loop has one free-integrator slope at low frequency.",
      },
      "unknown-c": {
        label: "Unknown loop C",
        type: 2,
        constantDb: 8,
        corner: 0.55,
        note: "This loop has two free-integrator slopes at low frequency.",
      },
      "pi-shaped": {
        label: "PI-shaped low-frequency loop",
        type: 1,
        constantDb: 24,
        corner: 0.45,
        zero: 0.12,
        note: "The low-frequency portion has the extra integrator; above the PI zero the slope begins to flatten.",
      },
      "lag-shaped": {
        label: "Lag-shaped low-frequency loop",
        type: 0,
        constantDb: 34,
        corner: 0.35,
        lag: true,
        note: "The low-frequency gain is higher, but the low-frequency slope remains Type 0 because lag compensation does not add a free integrator.",
      },
      measured: {
        label: "Noisy measured loop",
        type: 1,
        constantDb: 18,
        corner: 0.8,
        ripple: true,
        note: "Small measurement ripple is present, so use the trend across a decade rather than one point.",
      },
    };
    const systemTypeLoopDbAt = (example, logOmega) => {
      const omega = 10 ** logOmega;
      let db = example.constantDb - 20 * example.type * logOmega;
      db -= 10 * Math.log10(1 + (omega / example.corner) ** 2);
      if (example.zero) db += 10 * Math.log10(1 + (omega / example.zero) ** 2);
      if (example.lag) db -= 9 * (omega ** 2 / (omega ** 2 + 0.09 ** 2));
      if (example.ripple) db += 1.1 * Math.sin(8.5 * logOmega + 0.6) + 0.55 * Math.sin(21 * logOmega);
      return db;
    };
    const drawSystemTypeCanvas = (activity, values) => {
      const canvas = activity.querySelector("[data-plot='system-type']");
      const ctx = canvas?.getContext?.("2d");
      if (!canvas || !ctx) return;
      const width = canvas.width;
      const height = canvas.height;
      const pad = { left: 76, right: 34, top: 42, bottom: 64 };
      const xMin = -3;
      const xMax = 0.2;
      const yMin = -30;
      const yMax = 140;
      const plotW = width - pad.left - pad.right;
      const plotH = height - pad.top - pad.bottom;
      const xMap = (logW) => pad.left + ((logW - xMin) / (xMax - xMin)) * plotW;
      const yMap = (db) => pad.top + ((yMax - db) / (yMax - yMin)) * plotH;
      const clamp = (db) => Math.max(yMin, Math.min(yMax, db));
      const logW1 = values.logW1;
      const logW2 = values.logW2;
      const x1 = xMap(logW1);
      const x2 = xMap(logW2);
      const y1 = yMap(clamp(values.db1));
      const y2 = yMap(clamp(values.db2));
      const guideDbAtLog = (logW) => -20 * values.inputOrder * logW;
      const guideY1 = yMap(clamp(guideDbAtLog(logW1)));
      const showGuide = values.classificationCorrect;

      const paintFallback = (message) => {
        ctx.clearRect(0, 0, width, height);
        ctx.fillStyle = "#fbfdfe";
        ctx.fillRect(0, 0, width, height);
        ctx.strokeStyle = "#dbe7ea";
        ctx.lineWidth = 2;
        ctx.strokeRect(24, 24, width - 48, height - 48);
        ctx.fillStyle = "#172d33";
        ctx.font = "700 18px sans-serif";
        ctx.fillText("System type plot unavailable", 48, 68);
        ctx.font = "14px sans-serif";
        ctx.fillText(message, 48, 100);
      };

      if (
        !Number.isFinite(values.actualType)
        || !Number.isFinite(values.inputOrder)
        || !Number.isFinite(values.logW1)
        || !Number.isFinite(values.logW2)
        || !Number.isFinite(values.db1)
        || !Number.isFinite(values.db2)
        || !Number.isFinite(values.slope)
        || !Number.isFinite(x1)
        || !Number.isFinite(x2)
        || !Number.isFinite(y1)
        || !Number.isFinite(y2)
      ) {
        paintFallback("One of the selected system-type values is not finite.");
        return;
      }

      const drawCurve = (fn, color, widthPx, dash = []) => {
        ctx.save();
        ctx.beginPath();
        ctx.rect(pad.left, pad.top, plotW, plotH);
        ctx.clip();
        ctx.beginPath();
        ctx.setLineDash(dash);
        let hasPoint = false;
        for (let i = 0; i <= 240; i += 1) {
          const logW = xMin + (i / 240) * (xMax - xMin);
          const db = fn(logW);
          if (!Number.isFinite(db)) continue;
          const x = xMap(logW);
          const y = yMap(db);
          if (!hasPoint) {
            ctx.moveTo(x, y);
            hasPoint = true;
          } else {
            ctx.lineTo(x, y);
          }
        }
        if (hasPoint) {
          ctx.strokeStyle = color;
          ctx.lineWidth = widthPx;
          ctx.stroke();
        }
        ctx.restore();
      };

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#fbfdfe";
      ctx.fillRect(0, 0, width, height);
      ctx.strokeStyle = "#dce8eb";
      ctx.lineWidth = 1;
      ctx.font = "12px sans-serif";
      ctx.fillStyle = "#526047";
      for (let logW = xMin; logW <= xMax; logW += 1) {
        const x = xMap(logW);
        ctx.beginPath();
        ctx.moveTo(x, pad.top);
        ctx.lineTo(x, pad.top + plotH);
        ctx.stroke();
        ctx.fillText(`10^${logW}`, x - 14, height - 30);
      }
      for (let db = yMin; db <= yMax; db += 20) {
        const y = yMap(db);
        ctx.beginPath();
        ctx.moveTo(pad.left, y);
        ctx.lineTo(pad.left + plotW, y);
        ctx.stroke();
        ctx.fillText(String(db), 28, y + 4);
      }
      ctx.strokeStyle = "#172d33";
      ctx.lineWidth = 1.8;
      ctx.strokeRect(pad.left, pad.top, plotW, plotH);
      ctx.fillStyle = "#172d33";
      ctx.font = "14px sans-serif";
      ctx.fillText("dB", 24, pad.top + 4);
      ctx.fillText("omega (rad/s)", width - 130, height - 16);
      ctx.font = "700 15px sans-serif";
      ctx.fillText(values.exampleLabel, pad.left, 24);

      drawCurve((logW) => systemTypeLoopDbAt(values.example, logW), "#0e6d77", 3);
      if (showGuide) drawCurve(guideDbAtLog, "#d56b35", 2.5, [7, 5]);

      ctx.strokeStyle = "#7b8790";
      ctx.setLineDash([7, 5]);
      ctx.beginPath();
      ctx.moveTo(x1, pad.top);
      ctx.lineTo(x1, pad.top + plotH);
      ctx.moveTo(x2, pad.top);
      ctx.lineTo(x2, pad.top + plotH);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.strokeStyle = "#6a7f19";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
      ctx.fillStyle = "#0e6d77";
      ctx.beginPath();
      ctx.arc(x1, y1, 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.beginPath();
      ctx.arc(x2, y2, 6, 0, Math.PI * 2);
      ctx.fill();

      if (showGuide) {
        ctx.strokeStyle = "#d56b35";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x1, guideY1);
        ctx.stroke();
      }

      ctx.fillStyle = "rgba(255,255,255,0.92)";
      ctx.strokeStyle = "#dbe7ea";
      ctx.lineWidth = 1;
      ctx.fillRect(width - 380, 48, 342, 122);
      ctx.strokeRect(width - 380, 48, 342, 122);
      ctx.font = "13px sans-serif";
      ctx.fillStyle = "#0e6d77";
      ctx.fillText("blue: measured loop gain |P(jomega)C(jomega)|", width - 360, 72);
      ctx.fillStyle = "#6a7f19";
      ctx.fillText(`green: marker slope = ${fmt(values.slope, 1)} dB/dec`, width - 360, 96);
      ctx.fillStyle = "#d56b35";
      ctx.fillText(showGuide ? `orange: ${values.inputLabel} guide` : "orange guide appears after correct type", width - 360, 120);
      ctx.fillStyle = "#172d33";
      ctx.fillText(showGuide ? `gap at omega1: ${fmt(values.gapDb, 1)} dB` : "first classify the low-frequency type", width - 360, 144);
    };

    const drawActivityVisual = (activity, type, values) => {
      if (type === "transfer-function") {
        const { a1, a0, b0, dc, polePoints } = values;
        const poleX = (value) => Math.max(372, Math.min(620, 496 + value * 32));
        const poleY = (value) => Math.max(34, Math.min(204, 120 - value * 28));
        const poleMarks = polePoints.map((pole) => {
          const x = poleX(pole.re);
          const y = poleY(pole.im);
          const label = pole.im === 0
            ? fmt(pole.re, 2)
            : `${fmt(pole.re, 2)} ${pole.im > 0 ? "+" : "-"} j${fmt(Math.abs(pole.im), 2)}`;
          return `
            <g>
              <line x1="${x - 7}" y1="${y - 7}" x2="${x + 7}" y2="${y + 7}" stroke="#d56b35" stroke-width="3"></line>
              <line x1="${x - 7}" y1="${y + 7}" x2="${x + 7}" y2="${y - 7}" stroke="#d56b35" stroke-width="3"></line>
              <text x="${Math.min(x + 10, 575)}" y="${Math.max(y - 9, 24)}" font-size="12" fill="#7b3f1f">${label}</text>
            </g>
          `;
        }).join("");
        drawAutoVisual(activity, "Transfer function block diagram and pole locations", "The denominator coefficients set the poles; the numerator sets the input-output scale.", `
          <g aria-label="Transfer function block diagram">
            ${arrow(20, 120, 78, 120)}
            <rect x="82" y="80" width="225" height="80" rx="10" fill="#f7fbfc" stroke="#0e6d77" stroke-width="3"></rect>
            <text x="118" y="126" text-anchor="middle" font-size="15" fill="#172d33">H(s) =</text>
            <text x="218" y="106" text-anchor="middle" font-size="15" fill="#172d33">${fmt(b0)}</text>
            <line x1="154" y1="116" x2="284" y2="116" stroke="#172d33" stroke-width="1.8"></line>
            <text x="218" y="139" text-anchor="middle" font-size="15" fill="#172d33">s² + ${fmt(a1)}s + ${fmt(a0)}</text>
            ${arrow(307, 120, 350, 120)}
            <text x="20" y="105" font-size="15">u(s)</text>
            <text x="320" y="105" font-size="15">y(s)</text>
            <text x="98" y="188" font-size="14" fill="#526047">DC gain H(0) = ${dc}</text>
          </g>
          <g aria-label="Pole plot">
            <text x="435" y="28" font-size="15" font-weight="700" fill="#244b54">Pole locations</text>
            <line x1="370" y1="120" x2="625" y2="120" stroke="#172d33" stroke-width="2"></line>
            <line x1="496" y1="205" x2="496" y2="34" stroke="#172d33" stroke-width="2"></line>
            <path d="M625 120 l-9 -5 v10 z" fill="#172d33"></path>
            <path d="M496 34 l-5 9 h10 z" fill="#172d33"></path>
            <text x="584" y="142" font-size="13" fill="#172d33">Real</text>
            <text x="506" y="48" font-size="13" fill="#172d33">Imag</text>
            <line x1="368" y1="116" x2="368" y2="124" stroke="#8ba4aa"></line>
            <text x="360" y="139" font-size="11" fill="#526047">-4</text>
            <line x1="496" y1="116" x2="496" y2="124" stroke="#8ba4aa"></line>
            <text x="491" y="139" font-size="11" fill="#526047">0</text>
            <line x1="560" y1="116" x2="560" y2="124" stroke="#8ba4aa"></line>
            <text x="556" y="139" font-size="11" fill="#526047">2</text>
            ${poleMarks}
          </g>
        `, "0 0 640 240");
      }
      if (type === "state-space") {
        const { a1, a0, b0, x1, x2, u, c1, c2, d, x1dot, x2dot, y } = values;
        const svgTerm = (coefficient, variable, first = false) => {
          if (coefficient === 0) return first ? "0" : "";
          const sign = coefficient < 0 ? (first ? "-" : " - ") : (first ? "" : " + ");
          return `${sign}${fmt(Math.abs(coefficient), 2)}${variable}`;
        };
        const odeLeft = `y''${svgTerm(a1, "y'")}${svgTerm(a0, "y")}`;
        const stateSecond = `x2' = ${svgTerm(-a0, "x1", true)}${svgTerm(-a1, "x2")}${svgTerm(b0, "u")}`;
        const outputEquation = `y = ${svgTerm(c1, "x1", true)}${svgTerm(c2, "x2")}${svgTerm(d, "u")}`;
        const dt = 0.04;
        const steps = 200;
        const trace = [];
        let sx1 = x1;
        let sx2 = x2;
        for (let i = 0; i <= steps; i += 1) {
          const t = i * dt;
          const sy = c1 * sx1 + c2 * sx2 + d * u;
          trace.push({ t, x1: sx1, x2: sx2, y: sy, u });
          const dx1 = sx2;
          const dx2 = -a0 * sx1 - a1 * sx2 + b0 * u;
          sx1 += dt * dx1;
          sx2 += dt * dx2;
          if (!Number.isFinite(sx1) || !Number.isFinite(sx2) || Math.abs(sx1) > 60 || Math.abs(sx2) > 60) break;
        }
        const plot = { left: 48, top: 158, right: 630, bottom: 310 };
        const tMax = steps * dt;
        const maxAbs = Math.max(
          1,
          ...trace.flatMap((point) => [Math.abs(point.x1), Math.abs(point.x2), Math.abs(point.y), Math.abs(point.u)])
        );
        const yMax = Math.min(60, Math.max(1, maxAbs * 1.12));
        const px = (t) => plot.left + (t / tMax) * (plot.right - plot.left);
        const py = (value) => {
          const clipped = Math.max(-yMax, Math.min(yMax, value));
          return plot.bottom - ((clipped + yMax) / (2 * yMax)) * (plot.bottom - plot.top);
        };
        const pathFor = (key) => trace.map((point, i) => `${i === 0 ? "M" : "L"}${fmt(px(point.t), 2)} ${fmt(py(point[key]), 2)}`).join(" ");
        drawAutoVisual(activity, "State-space state and output response", "A and B set the state trajectory. C and D only change the measured output y(t).", `
          <defs>
            <marker id="state-space-arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
              <path d="M0 0 L8 4 L0 8 Z" fill="#172d33"></path>
            </marker>
          </defs>
          <g aria-label="State-space equation map">
            <rect x="18" y="22" width="142" height="74" rx="10" fill="#ffffff" stroke="#0e6d77" stroke-width="2.5"></rect>
            <text x="89" y="49" text-anchor="middle" font-size="14" font-weight="700">ODE</text>
            <text x="89" y="74" text-anchor="middle" font-size="12">${odeLeft} = ${fmt(b0)}u</text>
            <line x1="160" y1="59" x2="190" y2="59" stroke="#172d33" stroke-width="2.5" marker-end="url(#state-space-arrow)"></line>
            <rect x="194" y="22" width="258" height="74" rx="10" fill="#f7fbfc" stroke="#0e6d77" stroke-width="2.5"></rect>
            <text x="323" y="48" text-anchor="middle" font-size="14" font-weight="700">State equation</text>
            <text x="323" y="73" text-anchor="middle" font-size="12">x1' = x2, ${stateSecond}</text>
            <line x1="452" y1="59" x2="482" y2="59" stroke="#172d33" stroke-width="2.5" marker-end="url(#state-space-arrow)"></line>
            <rect x="486" y="22" width="142" height="74" rx="10" fill="#fff8e8" stroke="#d56b35" stroke-width="2.5"></rect>
            <text x="557" y="48" text-anchor="middle" font-size="14" font-weight="700">Output equation</text>
            <text x="557" y="73" text-anchor="middle" font-size="12">${outputEquation}</text>
            <text x="36" y="126" font-size="13" fill="#526047">Current point: x = [${fmt(x1, 2)}, ${fmt(x2, 2)}], u = ${fmt(u, 2)}, x' = [${fmt(x1dot, 2)}, ${fmt(x2dot, 2)}], y = ${fmt(y, 2)}</text>
          </g>
          <g aria-label="State and output trajectory">
            <rect x="${plot.left}" y="${plot.top}" width="${plot.right - plot.left}" height="${plot.bottom - plot.top}" fill="#ffffff" stroke="#dbe7ea"></rect>
            <line x1="${plot.left}" y1="${py(0)}" x2="${plot.right}" y2="${py(0)}" stroke="#9fb3b9" stroke-width="1"></line>
            <line x1="${plot.left}" y1="${plot.bottom}" x2="${plot.right}" y2="${plot.bottom}" stroke="#172d33" stroke-width="2"></line>
            <line x1="${plot.left}" y1="${plot.top}" x2="${plot.left}" y2="${plot.bottom}" stroke="#172d33" stroke-width="2"></line>
            <path d="${pathFor("x1")}" fill="none" stroke="#0e6d77" stroke-width="3"></path>
            <path d="${pathFor("x2")}" fill="none" stroke="#6a7f19" stroke-width="3"></path>
            <path d="${pathFor("y")}" fill="none" stroke="#d56b35" stroke-width="3"></path>
            <path d="${pathFor("u")}" fill="none" stroke="#7b8790" stroke-width="2" stroke-dasharray="5 5"></path>
            <text x="${plot.left}" y="${plot.top - 14}" font-size="13" font-weight="700" fill="#244b54">Response for constant input over 8 seconds</text>
            <text x="${plot.right - 38}" y="${plot.bottom + 22}" font-size="12">t</text>
            <text x="${plot.left - 42}" y="${plot.top + 8}" font-size="12">+${fmt(yMax, 1)}</text>
            <text x="${plot.left - 36}" y="${py(0) + 4}" font-size="12">0</text>
            <text x="${plot.left - 42}" y="${plot.bottom + 4}" font-size="12">-${fmt(yMax, 1)}</text>
            <g transform="translate(392 126)">
              <rect x="0" y="0" width="238" height="22" rx="6" fill="rgba(255,255,255,0.82)" stroke="#dbe7ea"></rect>
              <line x1="10" y1="11" x2="32" y2="11" stroke="#0e6d77" stroke-width="3"></line><text x="38" y="15" font-size="12">x1</text>
              <line x1="72" y1="11" x2="94" y2="11" stroke="#6a7f19" stroke-width="3"></line><text x="100" y="15" font-size="12">x2</text>
              <line x1="134" y1="11" x2="156" y2="11" stroke="#d56b35" stroke-width="3"></line><text x="162" y="15" font-size="12">y</text>
              <line x1="190" y1="11" x2="212" y2="11" stroke="#7b8790" stroke-width="2" stroke-dasharray="5 5"></line><text x="218" y="15" font-size="12">u</text>
            </g>
          </g>
        `, "0 0 660 335");
      }
      if (type === "state-feedback") {
        const poles = values.poles || "";
        const p1 = values.p1 ?? 2;
        const p2 = values.p2 ?? 3;
        const originX = 360;
        const leftX = 50;
        const yAxis = 105;
        const xForPole = (magnitude) => originX - (Math.max(0, Math.min(8, magnitude)) / 8) * (originX - leftX);
        const x1 = xForPole(p1);
        const x2 = xForPole(p2);
        drawAutoVisual(activity, "Desired real-pole map", "The pole sliders move the desired real closed-loop pole locations across the plotted real axis.", `
          <line x1="${leftX}" y1="${yAxis}" x2="390" y2="${yAxis}" stroke="#172d33" stroke-width="2"></line>
          <line x1="${originX}" y1="28" x2="${originX}" y2="182" stroke="#172d33" stroke-width="2"></line>
          <text x="372" y="96" font-size="13">Re</text><text x="${originX + 8}" y="40" font-size="13">Im</text>
          <line x1="${leftX}" y1="${yAxis - 5}" x2="${leftX}" y2="${yAxis + 5}" stroke="#172d33" stroke-width="1.5"></line>
          <text x="${leftX - 8}" y="${yAxis + 22}" font-size="12">-8</text>
          <line x1="${xForPole(4)}" y1="${yAxis - 5}" x2="${xForPole(4)}" y2="${yAxis + 5}" stroke="#172d33" stroke-width="1.5"></line>
          <text x="${xForPole(4) - 8}" y="${yAxis + 22}" font-size="12">-4</text>
          <text x="${originX + 8}" y="${yAxis + 22}" font-size="12">0</text>
          <circle cx="${x1}" cy="${yAxis}" r="8" fill="#d56b35"></circle>
          <circle cx="${x2}" cy="${yAxis}" r="8" fill="#0e6d77"></circle>
          <text x="${Math.min(x1 + 10, 360)}" y="${yAxis - 12}" font-size="12" fill="#8a4727">-p1</text>
          <text x="${Math.min(x2 + 10, 360)}" y="${yAxis + 24}" font-size="12" fill="#0e6d77">-p2</text>
          <text x="42" y="188" font-size="14" fill="#526047">desired poles: ${poles}</text>
        `);
      }
      if (type === "pd-poles") {
        const poles = values.poles || "";
        const sigma = values.sigma ?? 2;
        const wd = values.wd ?? 2;
        const originX = 360;
        const leftX = 50;
        const yAxis = 105;
        const x = originX - (Math.max(0, Math.min(8, sigma)) / 8) * (originX - leftX);
        const yOffset = (Math.max(0, Math.min(8, wd)) / 8) * 72;
        drawAutoVisual(activity, "Desired pole map", "The pole sliders move the desired closed-loop pole locations in the complex plane.", `
          <line x1="${leftX}" y1="${yAxis}" x2="390" y2="${yAxis}" stroke="#172d33" stroke-width="2"></line>
          <line x1="${originX}" y1="24" x2="${originX}" y2="188" stroke="#172d33" stroke-width="2"></line>
          <text x="372" y="96" font-size="13">Re</text><text x="${originX + 8}" y="36" font-size="13">Im</text>
          <circle cx="${x}" cy="${yAxis - yOffset}" r="8" fill="#d56b35"></circle>
          <circle cx="${x}" cy="${yAxis + yOffset}" r="8" fill="#d56b35"></circle>
          <text x="42" y="188" font-size="14" fill="#526047">desired poles: ${poles}</text>
        `);
      }
      if (type === "digital-pid") {
        const { ts, e, ePrev, trap, diff } = values;
        const xPrev = 90;
        const xNow = 90 + Math.min(220, Math.max(40, ts * 900));
        const yPrev = 155 - ePrev * 45;
        const yNow = 155 - e * 45;
        drawAutoVisual(activity, "Sampled error update", "The sample time controls the spacing between stored error samples used by the discrete controller.", `
          <line x1="42" y1="155" x2="375" y2="155" stroke="#172d33" stroke-width="2"></line>
          <line x1="58" y1="30" x2="58" y2="170" stroke="#172d33" stroke-width="2"></line>
          <line x1="${xPrev}" y1="${yPrev}" x2="${xNow}" y2="${yNow}" stroke="#0e6d77" stroke-width="4"></line>
          <circle cx="${xPrev}" cy="${yPrev}" r="8" fill="#d56b35"></circle>
          <circle cx="${xNow}" cy="${yNow}" r="8" fill="#0e6d77"></circle>
          <text x="${xPrev - 22}" y="${yPrev - 12}" font-size="13">e[k-1]</text>
          <text x="${xNow - 14}" y="${yNow - 12}" font-size="13">e[k]</text>
          <text x="72" y="190" font-size="14">ΔI = ${fmt(trap, 3)}, derivative ≈ ${fmt(diff, 2)}</text>
        `);
      }
      if (type === "integrator-pole") {
        const { pole, disturbance, settling } = values;
        const x = 55 + Math.min(285, pole * 36);
        const bias = 150 - disturbance * 100;
        drawAutoVisual(activity, "Integrator pole tuning", "Moving the integrator pole left improves bias rejection but increases control effort.", `
          <line x1="40" y1="112" x2="380" y2="112" stroke="#172d33" stroke-width="2"></line>
          <text x="42" y="100" font-size="13">slow</text><text x="340" y="100" font-size="13">fast</text>
          <circle cx="${x}" cy="112" r="11" fill="#d56b35"></circle>
          <line x1="70" y1="${bias}" x2="350" y2="${bias}" stroke="#0e6d77" stroke-width="3" stroke-dasharray="6 5"></line>
          <text x="72" y="178" font-size="14">pole at -${fmt(pole)}, disturbance ${fmt(disturbance, 2)}, settling proxy ${settling}</text>
        `);
      }
      if (type === "observer") {
        const speed = values.speed ?? 4;
        const noise = values.noise ?? values.residual ?? 0.1;
        const x = 70 + Math.min(260, speed * 24);
        const amp = Math.min(60, Math.max(8, noise * 90));
        drawAutoVisual(activity, "Observer speed tradeoff", "Faster observer dynamics converge sooner, but can amplify noise or modeling mismatch.", `
          <line x1="54" y1="150" x2="370" y2="150" stroke="#172d33" stroke-width="2"></line>
          <path d="M60 140 C110 ${140 - amp}, 160 ${160 + amp}, 220 ${145 - amp / 2} S310 ${145 + amp / 2}, 365 135" fill="none" stroke="#0e6d77" stroke-width="3"></path>
          <circle cx="${x}" cy="150" r="10" fill="#d56b35"></circle>
          <text x="62" y="180" font-size="14">observer speed ${fmt(speed, 2)}</text>
          <text x="210" y="180" font-size="14">sensitivity proxy ${fmt(noise, 3)}</text>
        `);
      }
      if (type === "complex-response") {
        const { real, imag, mag, phase } = values;
        const x = 205 + real * 38;
        const y = 105 - imag * 38;
        drawAutoVisual(activity, "Complex number vector", "The real and imaginary sliders move the frequency-response vector and change its magnitude and phase.", `
          <line x1="40" y1="105" x2="380" y2="105" stroke="#172d33" stroke-width="2"></line>
          <line x1="205" y1="25" x2="205" y2="185" stroke="#172d33" stroke-width="2"></line>
          ${arrow(205, 105, x, y, "#0e6d77")}
          <circle cx="${x}" cy="${y}" r="7" fill="#d56b35"></circle>
          <text x="44" y="202" font-size="14">|G(jω)| = ${fmt(mag, 2)}, phase = ${fmt(phase, 1)} deg</text>
        `);
      }
      if (type === "sensitivity") {
        const { loopDb, sDb, tDb } = values;
        const lX = 70 + Math.min(280, Math.max(0, (loopDb + 30) * 4));
        drawAutoVisual(activity, "Sensitivity tradeoff", "High loop gain lowers sensitivity S but pushes complementary sensitivity T toward one.", `
          <line x1="55" y1="150" x2="365" y2="150" stroke="#172d33" stroke-width="2"></line>
          <circle cx="${lX}" cy="150" r="10" fill="#d56b35"></circle>
          <path d="M60 48 C130 58, 210 98, 360 142" fill="none" stroke="#0e6d77" stroke-width="4"></path>
          <path d="M60 142 C130 110, 210 72, 360 52" fill="none" stroke="#d56b35" stroke-width="4"></path>
          <text x="70" y="184" font-size="14">loop gain ${fmt(loopDb, 1)} dB; S ${sDb}; T ${tDb}</text>
        `);
      }
      if (type === "margins") {
        const gainShift = values.gainShift ?? 0;
        const lag = values.phaseLag ?? 40;
        const gainCrossover = values.gainCrossoverValue;
        const phaseCrossover = values.phaseCrossoverValue;
        const pmValue = values.pmValue;
        const gmValue = values.gmValue;
        const phaseAtGainCrossover = values.phaseAtGainCrossover;
        const magAtPhaseCrossover = values.magAtPhaseCrossover;
        const xMin = -1.1;
        const xMax = 2.2;
        const magMin = -80;
        const magMax = 50;
        const phaseMin = -240;
        const phaseMax = -60;
        const magDb = (omega) => (
          gainShift
          + 12
          - 20 * Math.log10(omega)
          - 10 * Math.log10(1 + (omega / 6) ** 2)
          - 10 * Math.log10(1 + (omega / 40) ** 2)
        );
        const phaseDeg = (omega) => (
          -90
          - lag
          - Math.atan(omega / 6) * 180 / Math.PI
          - Math.atan(omega / 40) * 180 / Math.PI
        );
        const magPlot = { left: 80, top: 82, width: 600, height: 145 };
        const phasePlot = { left: 80, top: 305, width: 600, height: 145 };
        const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
        const xMap = (omega) => magPlot.left + ((Math.log10(omega) - xMin) / (xMax - xMin)) * magPlot.width;
        const magY = (db) => magPlot.top + ((magMax - db) / (magMax - magMin)) * magPlot.height;
        const phaseY = (deg) => phasePlot.top + ((phaseMax - deg) / (phaseMax - phaseMin)) * phasePlot.height;
        const curvePath = (plot, fn, yFn) => Array.from({ length: 150 }, (_, i) => {
          const logW = xMin + (i / 149) * (xMax - xMin);
          const omega = 10 ** logW;
          return `${i === 0 ? "M" : "L"}${xMap(omega).toFixed(1)} ${yFn(fn(omega)).toFixed(1)}`;
        }).join(" ");
        const xTicks = [-1, 0, 1, 2];
        const magTicks = [-60, -30, 0, 30];
        const phaseTicks = [-220, -180, -140, -100];
        const verticalGrid = (plot) => xTicks.map((tick) => {
          const x = magPlot.left + ((tick - xMin) / (xMax - xMin)) * magPlot.width;
          return `
            <line x1="${x}" y1="${plot.top}" x2="${x}" y2="${plot.top + plot.height}" stroke="#edf2f2" stroke-width="1"></line>
            <text x="${x}" y="${plot.top + plot.height + 19}" text-anchor="middle" font-size="11" fill="#526047">10^${tick}</text>
          `;
        }).join("");
        const horizontalGrid = (plot, ticks, yFn) => ticks.map((tick) => {
          const y = yFn(tick);
          return `
            <line x1="${plot.left}" y1="${y}" x2="${plot.left + plot.width}" y2="${y}" stroke="#edf2f2" stroke-width="1"></line>
            <text x="${plot.left - 10}" y="${y + 4}" text-anchor="end" font-size="11" fill="#526047">${tick}</text>
          `;
        }).join("");
        const plotFrame = (plot, label, unit) => `
          <rect x="${plot.left}" y="${plot.top}" width="${plot.width}" height="${plot.height}" fill="#fff" stroke="#cbdadd" stroke-width="1.3"></rect>
          ${verticalGrid(plot)}
          ${label === "Gain" ? horizontalGrid(plot, magTicks, magY) : horizontalGrid(plot, phaseTicks, phaseY)}
          <text x="${plot.left}" y="${plot.top - 16}" font-size="15" font-weight="700" fill="#172d33">${label}</text>
          <text x="${plot.left - 58}" y="${plot.top + plot.height / 2}" transform="rotate(-90 ${plot.left - 58} ${plot.top + plot.height / 2})" text-anchor="middle" font-size="12" fill="#526047">${unit}</text>
          <text x="${plot.left + plot.width / 2}" y="${plot.top + plot.height + 42}" text-anchor="middle" font-size="12" fill="#526047">Frequency, omega (rad/s)</text>
        `;
        const drawPhaseMargin = Number.isFinite(gainCrossover) && Number.isFinite(pmValue) && Number.isFinite(phaseAtGainCrossover)
          ? (() => {
            const x = xMap(gainCrossover);
            const y180 = phaseY(-180);
            const yPhase = phaseY(phaseAtGainCrossover);
            const labelX = Math.min(x + 16, magPlot.left + magPlot.width - 110);
            return `
              <line x1="${x}" y1="${magPlot.top}" x2="${x}" y2="${phasePlot.top + phasePlot.height}" stroke="#172d33" stroke-width="1.4" stroke-dasharray="6 5"></line>
              <circle cx="${x}" cy="${magY(0)}" r="5.5" fill="#d56b35"></circle>
              <circle cx="${x}" cy="${yPhase}" r="5.5" fill="${pmValue > 35 ? "#0e6d77" : "#d56b35"}"></circle>
              <line x1="${x + 12}" y1="${y180}" x2="${x + 12}" y2="${yPhase}" stroke="#d56b35" stroke-width="4" marker-start="url(#margin-arrow)" marker-end="url(#margin-arrow)"></line>
              <text x="${labelX}" y="${phasePlot.top + 28}" font-size="12" fill="#172d33">gain crossover</text>
              <text x="${labelX}" y="${phasePlot.top + 45}" font-size="12" font-weight="700" fill="#d56b35">PM = ${values.pm}</text>
            `;
          })()
          : `<text x="${phasePlot.left + 18}" y="${phasePlot.top + 28}" font-size="13" fill="#9c4735">No gain crossover in the plotted range, so phase margin is not defined.</text>`;
        const drawGainMargin = Number.isFinite(phaseCrossover) && Number.isFinite(gmValue) && Number.isFinite(magAtPhaseCrossover)
          ? (() => {
            const x = xMap(phaseCrossover);
            const y0 = magY(0);
            const yMag = magY(magAtPhaseCrossover);
            const labelX = Math.max(magPlot.left + 12, Math.min(x - 120, magPlot.left + magPlot.width - 130));
            return `
              <line x1="${x}" y1="${magPlot.top}" x2="${x}" y2="${phasePlot.top + phasePlot.height}" stroke="#526047" stroke-width="1.4" stroke-dasharray="5 5"></line>
              <circle cx="${x}" cy="${phaseY(-180)}" r="5.5" fill="#526047"></circle>
              <circle cx="${x}" cy="${yMag}" r="5.5" fill="${gmValue > 6 ? "#0e6d77" : "#d56b35"}"></circle>
              <line x1="${x - 12}" y1="${y0}" x2="${x - 12}" y2="${yMag}" stroke="#d56b35" stroke-width="4" marker-start="url(#margin-arrow)" marker-end="url(#margin-arrow)"></line>
              <text x="${labelX}" y="${magPlot.top + 28}" font-size="12" fill="#172d33">phase crossover</text>
              <text x="${labelX}" y="${magPlot.top + 45}" font-size="12" font-weight="700" fill="#d56b35">GM = ${values.gm}</text>
            `;
          })()
          : `<text x="${magPlot.left + 18}" y="${magPlot.top + 28}" font-size="13" fill="#9c4735">No -180 deg phase crossover in the plotted range, so gain margin is infinite/not defined here.</text>`;
        drawAutoVisual(activity, "Stability margins", "The upper plot shows gain. The lower plot shows phase for the same loop shape.", `
          <defs>
            <marker id="margin-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
              <path d="M 0 0 L 10 5 L 0 10 z" fill="#d56b35"></path>
            </marker>
            <clipPath id="ch17-margin-mag-clip"><rect x="${magPlot.left}" y="${magPlot.top}" width="${magPlot.width}" height="${magPlot.height}" rx="10"></rect></clipPath>
            <clipPath id="ch17-margin-phase-clip"><rect x="${phasePlot.left}" y="${phasePlot.top}" width="${phasePlot.width}" height="${phasePlot.height}" rx="10"></rect></clipPath>
          </defs>
          <text x="34" y="30" font-size="18" font-weight="700" fill="#172d33">Bode margin measurements</text>
          <text x="34" y="50" font-size="13" fill="#526047">Both curves come from the same loop shape. Read phase margin at 0 dB crossover and gain margin at -180 deg crossover.</text>
          ${plotFrame(magPlot, "Gain", "Magnitude (dB)")}
          ${plotFrame(phasePlot, "Phase", "Phase (deg)")}
          <line x1="${magPlot.left}" y1="${magY(0)}" x2="${magPlot.left + magPlot.width}" y2="${magY(0)}" stroke="#7f8d93" stroke-width="1.4" stroke-dasharray="7 5"></line>
          <line x1="${phasePlot.left}" y1="${phaseY(-180)}" x2="${phasePlot.left + phasePlot.width}" y2="${phaseY(-180)}" stroke="#7f8d93" stroke-width="1.4" stroke-dasharray="7 5"></line>
          <path d="${curvePath(magPlot, magDb, magY)}" fill="none" stroke="#0e6d77" stroke-width="3" clip-path="url(#ch17-margin-mag-clip)"></path>
          <path d="${curvePath(phasePlot, phaseDeg, phaseY)}" fill="none" stroke="#526047" stroke-width="3" clip-path="url(#ch17-margin-phase-clip)"></path>
          <text x="${magPlot.left + 8}" y="${magY(0) - 6}" font-size="11" fill="#526047">0 dB</text>
          <text x="${phasePlot.left + 8}" y="${phaseY(-180) - 6}" font-size="11" fill="#526047">-180 deg</text>
          ${drawPhaseMargin}
          ${drawGainMargin}
          <rect x="34" y="504" width="692" height="34" rx="10" fill="#fff8e8" stroke="#e4c16d"></rect>
          <text x="54" y="526" font-size="13" fill="#57411d">The dashed vertical guides show where each margin is measured on both plots.</text>
        `, "0 0 760 560");
      }
      if (type === "ch17-phase-response") {
        const pm = values.pm ?? 60;
        const zeta = Math.max(0.18, Math.min(1.15, pm / 85));
        const wn = 1;
        const bode = { left: 82, top: 78, width: 748, height: 205 };
        const step = { left: 82, top: 360, width: 748, height: 205 };
        const logMin = -1;
        const logMax = 1;
        const dbMin = -45;
        const dbMax = 16;
        const timeMax = 10;
        const yMin = -0.15;
        const yMax = 1.55;
        const xBode = (omega) => bode.left + ((Math.log10(Math.max(omega, 1e-4)) - logMin) / (logMax - logMin)) * bode.width;
        const yBode = (db) => bode.top + ((dbMax - db) / (dbMax - dbMin)) * bode.height;
        const xStep = (t) => step.left + (t / timeMax) * step.width;
        const yStep = (y) => step.top + ((yMax - Math.max(yMin, Math.min(yMax, y))) / (yMax - yMin)) * step.height;
        const magDbFor = (damping, omega) => {
          const r = omega / wn;
          const mag = 1 / Math.sqrt((1 - r * r) ** 2 + (2 * damping * r) ** 2);
          return 20 * Math.log10(Math.max(mag, 1e-6));
        };
        const stepValueFor = (damping, t) => {
          if (damping >= 1) return 1 - Math.exp(-wn * t);
          const wd = wn * Math.sqrt(1 - damping * damping);
          return 1 - Math.exp(-damping * wn * t) * (Math.cos(wd * t) + (damping / Math.sqrt(1 - damping * damping)) * Math.sin(wd * t));
        };
        const path = (count, xFn, yFn, valueFn, xValueFn) => Array.from({ length: count }, (_, i) => {
          const u = i / (count - 1);
          const xValue = xValueFn(u);
          return `${i === 0 ? "M" : "L"}${fmt(xFn(xValue), 2)} ${fmt(yFn(valueFn(xValue)), 2)}`;
        }).join(" ");
        const bodePathFor = (damping) => path(260, xBode, yBode, (omega) => magDbFor(damping, omega), (u) => 10 ** (logMin + u * (logMax - logMin)));
        const stepPathFor = (damping) => path(260, xStep, yStep, (t) => stepValueFor(damping, t), (u) => u * timeMax);
        const referenceCurves = [45, 60, 80];
        const peakDb = zeta < 1 / Math.sqrt(2) ? 20 * Math.log10(1 / (2 * zeta * Math.sqrt(1 - zeta * zeta))) : 0;
        const peakOmega = zeta < 1 / Math.sqrt(2) ? wn * Math.sqrt(1 - 2 * zeta * zeta) : null;
        const overshoot = zeta < 1 ? 100 * Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta)) : 0;
        const peakTime = zeta < 1 ? Math.PI / (wn * Math.sqrt(1 - zeta * zeta)) : null;
        const peakY = 1 + overshoot / 100;
        const gridLine = (plot, orientation, value, label, mapFn) => {
          if (orientation === "x") {
            const x = mapFn(value);
            return `
              <line x1="${x}" y1="${plot.top}" x2="${x}" y2="${plot.top + plot.height}" stroke="#e7eff1" stroke-width="1"></line>
              <text x="${x}" y="${plot.top + plot.height + 22}" text-anchor="middle" font-size="12" fill="#526047">${label}</text>
            `;
          }
          const y = mapFn(value);
          return `
            <line x1="${plot.left}" y1="${y}" x2="${plot.left + plot.width}" y2="${y}" stroke="#e7eff1" stroke-width="1"></line>
            <text x="${plot.left - 12}" y="${y + 4}" text-anchor="end" font-size="12" fill="#526047">${label}</text>
          `;
        };
        const referencePaths = referenceCurves.map((referencePm) => {
          const referenceZeta = Math.max(0.18, Math.min(1.15, referencePm / 85));
          return `
            <path d="${bodePathFor(referenceZeta)}" fill="none" stroke="#7f8d93" stroke-width="2" opacity="0.33" clip-path="url(#ch17-phase-bode-clip)"></path>
            <path d="${stepPathFor(referenceZeta)}" fill="none" stroke="#7f8d93" stroke-width="2" opacity="0.33" clip-path="url(#ch17-phase-step-clip)"></path>
          `;
        }).join("");
        const referenceLabels = referenceCurves.map((referencePm, index) => {
          const referenceZeta = Math.max(0.18, Math.min(1.15, referencePm / 85));
          const labelOmega = 0.16 + index * 0.18;
          const labelTime = 1.2 + index * 1.2;
          return `
            <text x="${xBode(labelOmega) + 6}" y="${yBode(magDbFor(referenceZeta, labelOmega)) - 5}" font-size="11" fill="#7f8d93">PM ${referencePm} deg</text>
            <text x="${xStep(labelTime) + 6}" y="${yStep(stepValueFor(referenceZeta, labelTime)) - 5}" font-size="11" fill="#7f8d93">PM ${referencePm} deg</text>
          `;
        }).join("");
        const bodeMarker = peakOmega && peakDb > 0.15 ? `
          <line x1="${xBode(peakOmega)}" y1="${yBode(0)}" x2="${xBode(peakOmega)}" y2="${yBode(peakDb)}" stroke="#0e6d77" stroke-width="1.8" stroke-dasharray="5 4"></line>
          <circle cx="${xBode(peakOmega)}" cy="${yBode(peakDb)}" r="6" fill="#0e6d77"></circle>
          <text x="${Math.min(xBode(peakOmega) + 12, bode.left + bode.width - 135)}" y="${Math.max(yBode(peakDb) - 10, bode.top + 18)}" font-size="13" font-weight="700" fill="#0e6d77">M<tspan baseline-shift="sub" font-size="10">r</tspan> ≈ ${fmt(peakDb, 1)} dB</text>
        ` : `
          <text x="${bode.left + bode.width - 182}" y="${bode.top + 26}" font-size="13" font-weight="700" fill="#0e6d77">No resonant peak above 0 dB</text>
        `;
        const stepMarker = peakTime ? `
          <line x1="${xStep(peakTime)}" y1="${yStep(1)}" x2="${xStep(peakTime)}" y2="${yStep(peakY)}" stroke="#d56b35" stroke-width="1.8" stroke-dasharray="5 4"></line>
          <circle cx="${xStep(peakTime)}" cy="${yStep(peakY)}" r="6" fill="#d56b35"></circle>
          <text x="${Math.min(xStep(peakTime) + 12, step.left + step.width - 130)}" y="${Math.max(yStep(peakY) - 10, step.top + 18)}" font-size="13" font-weight="700" fill="#d56b35">M<tspan baseline-shift="sub" font-size="10">p</tspan> ≈ ${fmt(overshoot, 1)}%</text>
        ` : "";
        const qualityX = bode.left + bode.width - 286;
        const qualityLabel = pm < 45 ? "low margin: expect ringing" : pm <= 70 ? "typical design range" : "high margin: conservative damping";
        drawAutoVisual(activity, "Phase margin and closed-loop response", "The plots use a second-order approximation to show the qualitative effect of phase margin.", `
          <text x="34" y="34" font-size="18" font-weight="700" fill="#172d33">Phase margin changes peaking and ringing</text>
          <defs>
            <clipPath id="ch17-phase-bode-clip"><rect x="${bode.left}" y="${bode.top}" width="${bode.width}" height="${bode.height}" rx="10"></rect></clipPath>
            <clipPath id="ch17-phase-step-clip"><rect x="${step.left}" y="${step.top}" width="${step.width}" height="${step.height}" rx="10"></rect></clipPath>
          </defs>
          <rect x="${qualityX}" y="16" width="252" height="30" rx="15" fill="${pm < 45 ? "#fbe8df" : pm <= 70 ? "#e7f3ec" : "#eaf0f7"}" stroke="${pm < 45 ? "#d56b35" : pm <= 70 ? "#6aa06f" : "#7f8d93"}"></rect>
          <text x="${qualityX + 126}" y="36" text-anchor="middle" font-size="13" font-weight="700" fill="#244b54">${qualityLabel}</text>
          <rect x="${bode.left}" y="${bode.top}" width="${bode.width}" height="${bode.height}" rx="10" fill="#fff" stroke="#cbdadd"></rect>
          <rect x="${step.left}" y="${step.top}" width="${step.width}" height="${step.height}" rx="10" fill="#fff" stroke="#cbdadd"></rect>
          ${[-40, -30, -20, -10, 0, 10].map((tick) => gridLine(bode, "y", tick, `${tick}`, yBode)).join("")}
          ${[0.1, 1, 10].map((tick) => gridLine(bode, "x", tick, tick === 1 ? "omega_co" : `${fmt(tick, 1)} omega_co`, xBode)).join("")}
          ${[0, 0.5, 1, 1.5].map((tick) => gridLine(step, "y", tick, fmt(tick, 1), yStep)).join("")}
          ${[0, 2, 4, 6, 8, 10].map((tick) => gridLine(step, "x", tick, String(tick), xStep)).join("")}
          <line x1="${bode.left}" y1="${yBode(0)}" x2="${bode.left + bode.width}" y2="${yBode(0)}" stroke="#9fb3b9" stroke-dasharray="6 5" stroke-width="1.4"></line>
          <line x1="${step.left}" y1="${yStep(1)}" x2="${step.left + step.width}" y2="${yStep(1)}" stroke="#9fb3b9" stroke-dasharray="6 5" stroke-width="1.4"></line>
          ${referencePaths}
          <path d="${bodePathFor(zeta)}" fill="none" stroke="#0e6d77" stroke-width="4" clip-path="url(#ch17-phase-bode-clip)"></path>
          <path d="${stepPathFor(zeta)}" fill="none" stroke="#d56b35" stroke-width="4" clip-path="url(#ch17-phase-step-clip)"></path>
          ${referenceLabels}
          ${bodeMarker}
          ${stepMarker}
          <text x="${bode.left}" y="${bode.top - 16}" font-size="15" font-weight="700" fill="#244b54">Closed-loop magnitude |T(jomega)|</text>
          <text x="${step.left}" y="${step.top - 16}" font-size="15" font-weight="700" fill="#244b54">Closed-loop unit-step response</text>
          <text x="${bode.left - 48}" y="${bode.top + 10}" font-size="12" fill="#526047">dB</text>
          <text x="${step.left - 50}" y="${step.top + 10}" font-size="12" fill="#526047">y(t)</text>
          <text x="${bode.left + bode.width - 80}" y="${bode.top + bode.height + 42}" font-size="12" fill="#526047">omega / omega_co</text>
          <text x="${step.left + step.width - 112}" y="${step.top + step.height + 42}" font-size="12" fill="#526047">normalized time, omega_n t</text>
          <rect x="82" y="600" width="748" height="42" rx="12" fill="#fff8e8" stroke="#e4c16d"></rect>
          <text x="104" y="626" font-size="13" fill="#57411d">Approximation: zeta ≈ PM / 85. It shows the trend, but a full loop shape can deviate from this second-order model.</text>
        `, "0 0 900 660");
      }
      if (type === "ch17-crossover-effort") {
        const wc = values.wc ?? 10;
        const zeta = 0.68;
        const bode = { left: 82, top: 78, width: 748, height: 190 };
        const response = { left: 82, top: 356, width: 350, height: 170 };
        const effort = { left: 500, top: 356, width: 330, height: 170 };
        const logMin = Math.log10(0.5);
        const logMax = Math.log10(300);
        const dbMin = -50;
        const dbMax = 8;
        const timeMax = 6;
        const effortMax = 140;
        const xBode = (omega) => bode.left + ((Math.log10(Math.max(omega, 0.01)) - logMin) / (logMax - logMin)) * bode.width;
        const yBode = (db) => bode.top + ((dbMax - db) / (dbMax - dbMin)) * bode.height;
        const xResp = (t) => response.left + (t / timeMax) * response.width;
        const xEffort = (t) => effort.left + (t / timeMax) * effort.width;
        const yResp = (y) => response.top + ((1.35 - Math.max(-0.05, Math.min(1.35, y))) / 1.4) * response.height;
        const peakU = 0.65 * wc;
        const yEffort = (u) => effort.top + ((effortMax - Math.max(0, Math.min(effortMax, u))) / effortMax) * effort.height;
        const closedLoopMagDb = (omega, crossover) => {
          const r = omega / crossover;
          const mag = 1 / Math.sqrt((1 - r * r) ** 2 + (2 * zeta * r) ** 2);
          return 20 * Math.log10(Math.max(mag, 1e-6));
        };
        const yValue = (t, crossover) => {
          const wd = crossover * Math.sqrt(1 - zeta * zeta);
          return 1 - Math.exp(-zeta * crossover * t) * (Math.cos(wd * t) + (zeta / Math.sqrt(1 - zeta * zeta)) * Math.sin(wd * t));
        };
        const uValue = (t, crossover) => {
          const peak = 0.65 * crossover;
          return peak * Math.exp(-crossover * t / 2.2) * (0.58 + 0.42 * Math.cos(crossover * t));
        };
        const bodePathFor = (crossover) => Array.from({ length: 260 }, (_, i) => {
          const u = i / 259;
          const omega = 10 ** (logMin + u * (logMax - logMin));
          return `${i === 0 ? "M" : "L"}${fmt(xBode(omega), 2)} ${fmt(yBode(closedLoopMagDb(omega, crossover)), 2)}`;
        }).join(" ");
        const timePathFor = (plot, xFn, yFn, valueFn, crossover) => Array.from({ length: 260 }, (_, i) => {
          const t = (i / 259) * timeMax;
          return `${i === 0 ? "M" : "L"}${fmt(xFn(t), 2)} ${fmt(yFn(valueFn(t, crossover)), 2)}`;
        }).join(" ");
        const settling = 4 / (zeta * wc);
        const referenceCrossovers = [2, 20, 100];
        const referencePaths = referenceCrossovers.map((referenceWc) => `
          <path d="${bodePathFor(referenceWc)}" fill="none" stroke="#7f8d93" stroke-width="2" opacity="0.3" clip-path="url(#ch17-crossover-bode-clip)"></path>
          <path d="${timePathFor(response, xResp, yResp, yValue, referenceWc)}" fill="none" stroke="#7f8d93" stroke-width="2" opacity="0.3" clip-path="url(#ch17-crossover-response-clip)"></path>
          <path d="${timePathFor(effort, xEffort, yEffort, uValue, referenceWc)}" fill="none" stroke="#7f8d93" stroke-width="2" opacity="0.3" clip-path="url(#ch17-crossover-effort-clip)"></path>
        `).join("");
        const referenceLabels = referenceCrossovers.map((referenceWc, index) => {
          const labelOmega = referenceWc;
          const labelT = Math.min(5.4, 0.7 + index * 1.3);
          const labelUTime = Math.min(5.4, 0.2 + index * 1.1);
          return `
            <text x="${Math.min(xBode(labelOmega) + 8, bode.left + bode.width - 86)}" y="${Math.max(yBode(closedLoopMagDb(labelOmega, referenceWc)) - 8, bode.top + 18)}" font-size="11" fill="#7f8d93">ωco ${referenceWc}</text>
            <text x="${xResp(labelT) + 6}" y="${yResp(yValue(labelT, referenceWc)) - 5}" font-size="11" fill="#7f8d93">ωco ${referenceWc}</text>
            <text x="${xEffort(labelUTime) + 6}" y="${yEffort(uValue(labelUTime, referenceWc)) - 5}" font-size="11" fill="#7f8d93">ωco ${referenceWc}</text>
          `;
        }).join("");
        const gridLine = (plot, orientation, value, label, mapFn) => {
          if (orientation === "x") {
            const x = mapFn(value);
            return `
              <line x1="${x}" y1="${plot.top}" x2="${x}" y2="${plot.top + plot.height}" stroke="#e7eff1" stroke-width="1"></line>
              <text x="${x}" y="${plot.top + plot.height + 22}" text-anchor="middle" font-size="12" fill="#526047">${label}</text>
            `;
          }
          const y = mapFn(value);
          return `
            <line x1="${plot.left}" y1="${y}" x2="${plot.left + plot.width}" y2="${y}" stroke="#e7eff1" stroke-width="1"></line>
            <text x="${plot.left - 12}" y="${y + 4}" text-anchor="end" font-size="12" fill="#526047">${label}</text>
          `;
        };
        const effortLimit = 35;
        const effortWarning = peakU > effortLimit;
        const badgeLabel = effortWarning ? "actuator limit likely" : wc < 5 ? "slow but gentle" : "balanced tradeoff";
        const badgeFill = effortWarning ? "#fbe8df" : wc < 5 ? "#eaf0f7" : "#e7f3ec";
        const badgeStroke = effortWarning ? "#d56b35" : wc < 5 ? "#7f8d93" : "#6aa06f";
        const selectedBodeDb = closedLoopMagDb(wc, wc);
        const responseAtSettling = yValue(Math.min(settling, timeMax), wc);
        drawAutoVisual(activity, "Crossover frequency and control effort", "Higher crossover frequency moves bandwidth right, speeds up the response, and increases actuator demand.", `
          <text x="34" y="32" font-size="18" font-weight="700" fill="#172d33">Crossover frequency tradeoff</text>
          <defs>
            <clipPath id="ch17-crossover-bode-clip"><rect x="${bode.left}" y="${bode.top}" width="${bode.width}" height="${bode.height}" rx="10"></rect></clipPath>
            <clipPath id="ch17-crossover-response-clip"><rect x="${response.left}" y="${response.top}" width="${response.width}" height="${response.height}" rx="10"></rect></clipPath>
            <clipPath id="ch17-crossover-effort-clip"><rect x="${effort.left}" y="${effort.top}" width="${effort.width}" height="${effort.height}" rx="10"></rect></clipPath>
          </defs>
          <rect x="574" y="16" width="256" height="30" rx="15" fill="${badgeFill}" stroke="${badgeStroke}"></rect>
          <text x="702" y="36" text-anchor="middle" font-size="13" font-weight="700" fill="#244b54">${badgeLabel}</text>
          <rect x="${bode.left}" y="${bode.top}" width="${bode.width}" height="${bode.height}" rx="10" fill="#fff" stroke="#cbdadd"></rect>
          <rect x="${response.left}" y="${response.top}" width="${response.width}" height="${response.height}" rx="10" fill="#fff" stroke="#cbdadd"></rect>
          <rect x="${effort.left}" y="${effort.top}" width="${effort.width}" height="${effort.height}" rx="10" fill="#fff" stroke="#cbdadd"></rect>
          ${[-40, -20, 0].map((tick) => gridLine(bode, "y", tick, `${tick}`, yBode)).join("")}
          ${[1, 10, 100].map((tick) => gridLine(bode, "x", tick, `${tick}`, xBode)).join("")}
          ${[0, 0.5, 1].map((tick) => gridLine(response, "y", tick, fmt(tick, 1), yResp)).join("")}
          ${[0, 2, 4, 6].map((tick) => gridLine(response, "x", tick, String(tick), xResp)).join("")}
          ${[0, 35, 70, 105, 140].map((tick) => gridLine(effort, "y", tick, String(tick), yEffort)).join("")}
          ${[0, 2, 4, 6].map((tick) => gridLine(effort, "x", tick, String(tick), xEffort)).join("")}
          <line x1="${bode.left}" y1="${yBode(0)}" x2="${bode.left + bode.width}" y2="${yBode(0)}" stroke="#9fb3b9" stroke-dasharray="6 5" stroke-width="1.4"></line>
          <line x1="${response.left}" y1="${yResp(1)}" x2="${response.left + response.width}" y2="${yResp(1)}" stroke="#9fb3b9" stroke-dasharray="6 5" stroke-width="1.4"></line>
          <line x1="${effort.left}" y1="${yEffort(effortLimit)}" x2="${effort.left + effort.width}" y2="${yEffort(effortLimit)}" stroke="#9c4735" stroke-dasharray="6 5" stroke-width="1.4"></line>
          ${referencePaths}
          <path d="${bodePathFor(wc)}" fill="none" stroke="#0e6d77" stroke-width="4" clip-path="url(#ch17-crossover-bode-clip)"></path>
          <path d="${timePathFor(response, xResp, yResp, yValue, wc)}" fill="none" stroke="#0e6d77" stroke-width="4" clip-path="url(#ch17-crossover-response-clip)"></path>
          <path d="${timePathFor(effort, xEffort, yEffort, uValue, wc)}" fill="none" stroke="#d56b35" stroke-width="4" clip-path="url(#ch17-crossover-effort-clip)"></path>
          ${referenceLabels}
          <line x1="${xBode(wc)}" y1="${bode.top}" x2="${xBode(wc)}" y2="${bode.top + bode.height}" stroke="#172d33" stroke-width="1.8" stroke-dasharray="5 4"></line>
          <circle cx="${xBode(wc)}" cy="${yBode(selectedBodeDb)}" r="6" fill="#0e6d77"></circle>
          <text x="${Math.min(xBode(wc) + 12, bode.left + bode.width - 166)}" y="${Math.max(yBode(selectedBodeDb) - 10, bode.top + 18)}" font-size="13" font-weight="700" fill="#0e6d77">bandwidth near ωco = ${fmt(wc, 2)}</text>
          <line x1="${xResp(Math.min(settling, timeMax))}" y1="${response.top}" x2="${xResp(Math.min(settling, timeMax))}" y2="${response.top + response.height}" stroke="#172d33" stroke-width="1.6" stroke-dasharray="5 4"></line>
          <circle cx="${xResp(Math.min(settling, timeMax))}" cy="${yResp(responseAtSettling)}" r="6" fill="#0e6d77"></circle>
          <text x="${Math.min(xResp(Math.min(settling, timeMax)) + 10, response.left + response.width - 120)}" y="${response.top + 24}" font-size="13" font-weight="700" fill="#0e6d77">settles ≈ ${fmt(settling, 2)} s</text>
          <circle cx="${xEffort(0)}" cy="${yEffort(peakU)}" r="6" fill="#d56b35"></circle>
          <text x="${xEffort(0) + 12}" y="${Math.max(yEffort(peakU) - 10, effort.top + 18)}" font-size="13" font-weight="700" fill="#d56b35">peak ≈ ${fmt(peakU, 1)}</text>
          <text x="${effort.left + effort.width - 104}" y="${yEffort(effortLimit) - 8}" font-size="12" fill="#9c4735">example limit</text>
          <text x="${bode.left}" y="${bode.top - 16}" font-size="15" font-weight="700" fill="#244b54">Closed-loop magnitude |T(jω)|</text>
          <text x="${response.left}" y="${response.top - 16}" font-size="15" font-weight="700" fill="#244b54">Output response</text>
          <text x="${effort.left}" y="${effort.top - 16}" font-size="15" font-weight="700" fill="#244b54">Control effort proxy</text>
          <text x="${bode.left - 48}" y="${bode.top + 10}" font-size="12" fill="#526047">dB</text>
          <text x="${response.left - 50}" y="${response.top + 10}" font-size="12" fill="#526047">y(t)</text>
          <text x="${effort.left - 52}" y="${effort.top + 10}" font-size="12" fill="#526047">|u(t)|</text>
          <text x="${bode.left + bode.width - 96}" y="${bode.top + bode.height + 42}" font-size="12" fill="#526047">ω (rad/s)</text>
          <text x="${response.left + response.width - 58}" y="${response.top + response.height + 42}" font-size="12" fill="#526047">time (s)</text>
          <text x="${effort.left + effort.width - 58}" y="${effort.top + effort.height + 42}" font-size="12" fill="#526047">time (s)</text>
          <rect x="82" y="590" width="748" height="48" rx="12" fill="#fff8e8" stroke="#e4c16d"></rect>
          <text x="104" y="612" font-size="13" fill="#57411d">Teaching model: phase margin is held roughly fixed. Raising ωco shifts bandwidth right and compresses time, but peak actuator demand grows in proportion to ωco.</text>
          <text x="104" y="632" font-size="13" fill="#57411d">Gray curves show reference choices ωco = 2, 20, and 100 rad/s; the colored curves show the slider choice.</text>
        `, "0 0 900 660");
      }
      if (type === "ch17-loopshape-checklist") {
        const lowGain = values.lowGain ?? 50;
        const wc = values.wc ?? 1.4;
        const slopeMag = values.slopeMag ?? 40;
        const rolloff = values.rolloff ?? 25;
        const active = values.active || "low";
        const pm = Math.max(15, Math.min(95, 120 - 1.5 * slopeMag));
        const intFmt = (value) => String(Math.round(value));
        const magPlot = { left: 86, top: 72, width: 760, height: 265 };
        const phasePlot = { left: 86, top: 420, width: 760, height: 205 };
        const logMin = -2;
        const logMax = 2;
        const dbMin = -120;
        const dbMax = 60;
        const phaseMin = -270;
        const phaseMax = -90;
        const xMap = (omega) => magPlot.left + ((Math.log10(Math.max(omega, 1e-5)) - logMin) / (logMax - logMin)) * magPlot.width;
        const magY = (db) => magPlot.top + ((dbMax - db) / (dbMax - dbMin)) * magPlot.height;
        const phaseY = (deg) => phasePlot.top + ((phaseMax - deg) / (phaseMax - phaseMin)) * phasePlot.height;
        const clampLoopshape = (value, min, max) => Math.max(min, Math.min(max, value));
        const magDb = (omega) => {
          const x = Math.log10(omega / wc);
          if (x < -1) {
            const blend = Math.min(1, Math.max(0, (-x - 1) / 1.2));
            return slopeMag + (lowGain - slopeMag) * blend;
          }
          if (x <= 1) return -slopeMag * x;
          return -slopeMag - (slopeMag + rolloff) * (x - 1);
        };
        const phaseDeg = (omega) => {
          const x = Math.log10(omega / wc);
          const targetLogistic = Math.max(0.03, Math.min(0.97, (90 - pm) / 180));
          const center = Math.log(targetLogistic / (1 - targetLogistic)) / 2;
          return -90 - 180 / (1 + Math.exp(-2 * (x - center)));
        };
        const pathFor = (fn, yFn, count = 280) => Array.from({ length: count }, (_, i) => {
          const logW = logMin + (i / (count - 1)) * (logMax - logMin);
          const omega = 10 ** logW;
          return `${i === 0 ? "M" : "L"}${fmt(xMap(omega), 2)} ${fmt(yFn(fn(omega)), 2)}`;
        }).join(" ");
        const gridLine = (plot, orientation, value, label, mapFn) => {
          if (orientation === "x") {
            const x = xMap(value);
            return `
              <line x1="${x}" y1="${plot.top}" x2="${x}" y2="${plot.top + plot.height}" stroke="#e7eff1" stroke-width="1"></line>
              <text x="${x}" y="${plot.top + plot.height + 20}" text-anchor="middle" font-size="12" fill="#526047">${label}</text>
            `;
          }
          const y = mapFn(value);
          return `
            <line x1="${plot.left}" y1="${y}" x2="${plot.left + plot.width}" y2="${y}" stroke="#e7eff1" stroke-width="1"></line>
            <text x="${plot.left - 12}" y="${y + 4}" text-anchor="end" font-size="12" fill="#526047">${label}</text>
          `;
        };
        const wcX = xMap(wc);
        const slopeLeftX = xMap(wc / 10);
        const slopeRightX = xMap(wc * 10);
        const phaseAtWc = phaseDeg(wc);
        const lowDb = magDb(0.01);
        const noiseDb = magDb(100);
        const lowOk = lowDb >= 35;
        const slopeOk = slopeMag >= 20 && slopeMag <= 40;
        const effortOk = wc <= 2.5;
        const noiseOk = noiseDb <= -40;
        const statusColor = (ok) => ok ? "#337a4c" : "#9c4735";
        const statusFill = (ok) => ok ? "#eff9f1" : "#fff3ee";
        const activeRegion = {
          low: `<rect x="${magPlot.left}" y="${magPlot.top}" width="${Math.max(0, slopeLeftX - magPlot.left)}" height="${magPlot.height}" fill="#dceff2" opacity="0.52"></rect>`,
          slope: `<rect x="${slopeLeftX}" y="${magPlot.top}" width="${Math.max(0, slopeRightX - slopeLeftX)}" height="${magPlot.height}" fill="#fff0df" opacity="0.64"></rect>`,
          effort: `<rect x="${wcX - 12}" y="${magPlot.top}" width="24" height="${magPlot.height}" fill="#fbe8df" opacity="0.8"></rect>`,
          noise: `<rect x="${slopeRightX}" y="${magPlot.top}" width="${Math.max(0, magPlot.left + magPlot.width - slopeRightX)}" height="${magPlot.height}" fill="#eef2d7" opacity="0.62"></rect>`,
        }[active] || "";
        const badge = (x, y, n, label, ok, key) => `
          <g opacity="${active === key ? "1" : "0.78"}">
            <circle cx="${x}" cy="${y}" r="15" fill="${active === key ? statusColor(ok) : "#ffffff"}" stroke="${statusColor(ok)}" stroke-width="2.4"></circle>
            <text x="${x}" y="${y + 5}" text-anchor="middle" font-size="13" font-weight="800" fill="${active === key ? "#fff" : statusColor(ok)}">${n}</text>
            <rect x="${x + 22}" y="${y - 14}" width="${label.length * 6.5 + 54}" height="28" rx="14" fill="${statusFill(ok)}" stroke="${statusColor(ok)}"></rect>
            <text x="${x + 36}" y="${y + 5}" font-size="12" font-weight="700" fill="${statusColor(ok)}">${ok ? "PASS" : "ADJUST"}: ${label}</text>
          </g>
        `;
        const pmArrowX = wcX + 28;
        drawAutoVisual(activity, "Ideal loop-shape checklist", "Interactive ideal loop-shape checklist.", `
          <defs>
            <clipPath id="ch17-loopshape-mag-clip"><rect x="${magPlot.left}" y="${magPlot.top}" width="${magPlot.width}" height="${magPlot.height}" rx="10"></rect></clipPath>
            <clipPath id="ch17-loopshape-phase-clip"><rect x="${phasePlot.left}" y="${phasePlot.top}" width="${phasePlot.width}" height="${phasePlot.height}" rx="10"></rect></clipPath>
            <marker id="ch17-loopshape-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
              <path d="M 0 0 L 10 5 L 0 10 z" fill="#d56b35"></path>
            </marker>
          </defs>
          <text x="34" y="34" font-size="18" font-weight="700" fill="#172d33">Ideal loop shape: connect the checklist to the Bode plot</text>
          <rect x="${magPlot.left}" y="${magPlot.top}" width="${magPlot.width}" height="${magPlot.height}" rx="10" fill="#fff" stroke="#cbdadd"></rect>
          <rect x="${phasePlot.left}" y="${phasePlot.top}" width="${phasePlot.width}" height="${phasePlot.height}" rx="10" fill="#fff" stroke="#cbdadd"></rect>
          ${activeRegion}
          ${[0.01, 0.1, 1, 10, 100].map((tick) => gridLine(magPlot, "x", tick, `${tick}`, xMap)).join("")}
          ${[0.01, 0.1, 1, 10, 100].map((tick) => gridLine(phasePlot, "x", tick, `${tick}`, xMap)).join("")}
          ${[-120, -80, -40, 0, 40].map((tick) => gridLine(magPlot, "y", tick, `${tick}`, magY)).join("")}
          ${[-270, -225, -180, -135, -90].map((tick) => gridLine(phasePlot, "y", tick, `${tick}`, phaseY)).join("")}
          <line x1="${magPlot.left}" y1="${magY(0)}" x2="${magPlot.left + magPlot.width}" y2="${magY(0)}" stroke="#7f8d93" stroke-width="1.4" stroke-dasharray="7 5"></line>
          <line x1="${phasePlot.left}" y1="${phaseY(-180)}" x2="${phasePlot.left + phasePlot.width}" y2="${phaseY(-180)}" stroke="#7f8d93" stroke-width="1.4" stroke-dasharray="7 5"></line>
          <rect x="${slopeLeftX}" y="${magPlot.top + 12}" width="${Math.max(0, slopeRightX - slopeLeftX)}" height="34" rx="17" fill="none" stroke="#d56b35" stroke-width="3" stroke-dasharray="9 6"></rect>
          <text x="${(slopeLeftX + slopeRightX) / 2}" y="${magPlot.top + 35}" text-anchor="middle" font-size="13" font-weight="700" fill="#9c4735">about one decade on each side of ωco</text>
          <line x1="${wcX}" y1="${magPlot.top}" x2="${wcX}" y2="${phasePlot.top + phasePlot.height}" stroke="#172d33" stroke-width="2.2" stroke-dasharray="8 7"></line>
          <text x="${wcX}" y="${magPlot.top - 14}" text-anchor="middle" font-size="14" font-weight="700" fill="#172d33">ω<tspan baseline-shift="sub" font-size="10">co</tspan> = ${fmt(wc, 2)}</text>
          <path d="${pathFor(magDb, magY)}" fill="none" stroke="#0e6d77" stroke-width="4" clip-path="url(#ch17-loopshape-mag-clip)"></path>
          <path d="${pathFor(phaseDeg, phaseY)}" fill="none" stroke="#0e6d77" stroke-width="4" clip-path="url(#ch17-loopshape-phase-clip)"></path>
          <circle cx="${wcX}" cy="${magY(0)}" r="6" fill="#d56b35"></circle>
          <circle cx="${wcX}" cy="${phaseY(phaseAtWc)}" r="6" fill="#d56b35"></circle>
          <line x1="${pmArrowX}" y1="${phaseY(-180)}" x2="${pmArrowX}" y2="${phaseY(phaseAtWc)}" stroke="#d56b35" stroke-width="4" marker-start="url(#ch17-loopshape-arrow)" marker-end="url(#ch17-loopshape-arrow)"></line>
          <text x="${pmArrowX + 14}" y="${(phaseY(-180) + phaseY(phaseAtWc)) / 2 + 4}" font-size="15" font-weight="700" fill="#d56b35">PM ≈ ${intFmt(pm)}°</text>
          ${badge(magPlot.left + 30, clampLoopshape(magY(Math.min(lowDb, 48)), magPlot.top + 30, magPlot.top + magPlot.height - 30), "1", "tracking/disturbance", lowOk, "low")}
          ${badge((slopeLeftX + slopeRightX) / 2 - 78, magPlot.top + 72, "2", "shallow slope", slopeOk, "slope")}
          ${badge(wcX + 34, magPlot.top + magPlot.height - 46, "3", "actuator effort", effortOk, "effort")}
          ${badge(Math.min(magPlot.left + magPlot.width - 206, slopeRightX + 28), clampLoopshape(Math.max(magPlot.top + 78, magY(noiseDb)), magPlot.top + 30, magPlot.top + magPlot.height - 30), "4", "noise attenuation", noiseOk, "noise")}
          <text x="${magPlot.left}" y="${magPlot.top - 16}" font-size="15" font-weight="700" fill="#244b54">Loop magnitude |P(jω)C(jω)|</text>
          <text x="${phasePlot.left}" y="${phasePlot.top - 16}" font-size="15" font-weight="700" fill="#244b54">Loop phase ∠P(jω)C(jω)</text>
          <text x="${magPlot.left - 52}" y="${magPlot.top + 12}" font-size="12" fill="#526047">dB</text>
          <text x="${phasePlot.left - 58}" y="${phasePlot.top + 12}" font-size="12" fill="#526047">deg</text>
          <text x="${magPlot.left + magPlot.width - 92}" y="${magPlot.top + magPlot.height + 44}" font-size="12" fill="#526047">ω (rad/s)</text>
          <text x="${phasePlot.left + phasePlot.width - 92}" y="${phasePlot.top + phasePlot.height + 44}" font-size="12" fill="#526047">ω (rad/s)</text>
          <rect x="86" y="658" width="760" height="42" rx="12" fill="#fff8e8" stroke="#e4c16d"></rect>
          <text x="108" y="684" font-size="13" fill="#57411d">The best loop shape keeps |PC| large at low frequency, crosses 0 dB with a shallow slope and useful PM, limits ωco for saturation, and rolls off at high frequency.</text>
        `, "0 0 900 720");
      }
      if (type === "compensator-score") {
        drawCh18CompensatorDashboard(activity, values.params, values.metrics);
      }
      if (type === "ch18-slc") {
        const step = values.step || "inner";
        const separation = Number.isFinite(values.separation) ? values.separation : 10;
        const svg = activity.querySelector("[data-figure='ch18-slc-map']");
        const activeColor = {
          inner: "#0e6d77",
          collapse: "#8b5e24",
          outer: "#d56b35",
          prefilter: "#526047",
        }[step] || "#0e6d77";
        const muted = "#9aa7ad";
        const good = separation >= 8;
        const okay = separation >= 5;
        const sepColor = good ? "#0e6d77" : okay ? "#8b5e24" : "#9c4735";
        const sepWidth = Math.max(24, Math.min(240, (separation / 20) * 240));
        const strokeFor = (key) => {
          if (step === "inner") return key === "inner" ? activeColor : muted;
          if (step === "collapse") return key === "inner" || key === "equiv" ? activeColor : muted;
          if (step === "outer") return key === "outer" || key === "equiv" || key === "pout" ? activeColor : muted;
          if (step === "prefilter") return key === "prefilter" ? activeColor : muted;
          return muted;
        };
        const opacityFor = (key) => strokeFor(key) === muted ? 0.52 : 1;
        const arrowLine = (x1, y1, x2, y2, color = "#172d33", dash = "") => `
          <line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${color}" stroke-width="3" ${dash ? `stroke-dasharray="${dash}"` : ""} marker-end="url(#slc-arrow)"></line>
        `;
        const feedbackPath = (d, color, dash = "") => `
          <path d="${d}" fill="none" stroke="${color}" stroke-width="2.6" ${dash ? `stroke-dasharray="${dash}"` : ""} marker-end="url(#slc-arrow)"></path>
        `;
        const signalLabel = (x, y, text, color = "#526047") => `<text x="${x}" y="${y}" font-size="13" fill="${color}">${text}</text>`;
        const blockNode = (key, x, y, w, h, label, sub = "", fill = "#fff") => `
          <g opacity="${opacityFor(key)}">
            <rect x="${x}" y="${y}" width="${w}" height="${h}" rx="12" fill="${fill}" stroke="${strokeFor(key)}" stroke-width="4"></rect>
            <text x="${x + w / 2}" y="${y + h / 2 - (sub ? 2 : -5)}" text-anchor="middle" font-size="16" font-weight="700" fill="#172d33">${label}</text>
            ${sub ? `<text x="${x + w / 2}" y="${y + h / 2 + 22}" text-anchor="middle" font-size="12" fill="#526047">${sub}</text>` : ""}
          </g>
        `;
        const collapseOverlay = step === "collapse" || step === "outer" || step === "prefilter"
          ? `
            <g>
              <rect x="555" y="247" width="268" height="84" rx="14" fill="#fff8e8" stroke="${strokeFor("equiv")}" stroke-width="4"></rect>
              <text x="689" y="276" text-anchor="middle" font-size="14" font-weight="700" fill="#172d33">closed inner-loop dynamics</text>
              <text x="689" y="301" text-anchor="middle" font-size="14" fill="#57411d">G_in,cl = P_in C_in / (1 + P_in C_in)</text>
              <path d="M 700 185 C 740 208, 750 232, 720 247" fill="none" stroke="#8b5e24" stroke-width="3" stroke-dasharray="7 5" marker-end="url(#slc-arrow)"></path>
            </g>
          `
          : "";
        if (svg) {
          svg.setAttribute("viewBox", "0 0 900 520");
          svg.innerHTML = `
            <defs>
              <marker id="slc-arrow" markerWidth="9" markerHeight="9" refX="8" refY="4.5" orient="auto">
                <path d="M0 0 L9 4.5 L0 9 Z" fill="#172d33"></path>
              </marker>
            </defs>
            <rect x="0" y="0" width="900" height="520" rx="18" fill="#fbfdfe"></rect>
            <text x="30" y="34" font-size="20" font-weight="700" fill="#172d33">Successive loop closure signal map</text>
            <text x="30" y="58" font-size="13" fill="#526047">Trace the command through the outer loop, close the fast inner loop, then collapse it into the plant used for outer-loop design.</text>

            <rect x="28" y="78" width="124" height="34" rx="17" fill="${step === "inner" ? "#e4f4f2" : "#fff"}" stroke="${step === "inner" ? "#0e6d77" : "#dbe7ea"}"></rect>
            <text x="90" y="100" text-anchor="middle" font-size="12" font-weight="700" fill="#172d33">1 inner</text>
            <rect x="162" y="78" width="132" height="34" rx="17" fill="${step === "collapse" ? "#fff8e8" : "#fff"}" stroke="${step === "collapse" ? "#8b5e24" : "#dbe7ea"}"></rect>
            <text x="228" y="100" text-anchor="middle" font-size="12" font-weight="700" fill="#172d33">2 collapse</text>
            <rect x="304" y="78" width="124" height="34" rx="17" fill="${step === "outer" ? "#fff1ea" : "#fff"}" stroke="${step === "outer" ? "#d56b35" : "#dbe7ea"}"></rect>
            <text x="366" y="100" text-anchor="middle" font-size="12" font-weight="700" fill="#172d33">3 outer</text>
            <rect x="438" y="78" width="124" height="34" rx="17" fill="${step === "prefilter" ? "#eef4dd" : "#fff"}" stroke="${step === "prefilter" ? "#526047" : "#dbe7ea"}"></rect>
            <text x="500" y="100" text-anchor="middle" font-size="12" font-weight="700" fill="#172d33">4 prefilter</text>

            <circle cx="54" cy="180" r="7" fill="#172d33"></circle>
            ${signalLabel(34, 160, "r")}
            ${arrowLine(61, 180, 112, 180, step === "prefilter" ? activeColor : "#172d33")}
            ${blockNode("prefilter", 112, 142, 86, 76, "F(s)", "prefilter")}
            ${signalLabel(136, 132, "r_f", strokeFor("prefilter"))}
            ${arrowLine(198, 180, 252, 180, step === "prefilter" ? activeColor : "#172d33")}
            <circle cx="270" cy="180" r="18" fill="#fff" stroke="${strokeFor("outer")}" stroke-width="3"></circle>
            <text x="270" y="186" text-anchor="middle" font-size="18" fill="#172d33">Σ</text>
            ${signalLabel(250, 143, "e_out", strokeFor("outer"))}
            ${arrowLine(288, 180, 342, 180, strokeFor("outer"))}
            ${blockNode("outer", 342, 142, 104, 76, "C_out(s)", "slow loop")}
            ${signalLabel(368, 132, "u_out", strokeFor("outer"))}
            ${arrowLine(446, 180, 496, 180, strokeFor("pout"))}
            ${blockNode("pout", 496, 142, 98, 76, "P_out(s)", "outer plant")}
            ${signalLabel(512, 132, "r_in", strokeFor("pout"))}
            ${arrowLine(594, 180, 638, 180, strokeFor("inner"))}
            <circle cx="656" cy="180" r="18" fill="#fff" stroke="${strokeFor("inner")}" stroke-width="3"></circle>
            <text x="656" y="186" text-anchor="middle" font-size="18" fill="#172d33">Σ</text>
            ${signalLabel(635, 143, "e_in", strokeFor("inner"))}
            ${arrowLine(674, 180, 724, 180, strokeFor("inner"))}
            ${blockNode("inner", 724, 142, 96, 76, "C_in(s)", "fast loop")}
            ${signalLabel(748, 132, "u", strokeFor("inner"))}
            ${arrowLine(820, 180, 850, 180, strokeFor("inner"))}
            ${blockNode("inner", 738, 270, 96, 76, "P_in(s)", "inner plant")}
            ${feedbackPath("M 850 180 C 870 205, 870 292, 834 308", strokeFor("inner"))}
            ${feedbackPath("M 738 308 C 694 308, 656 258, 656 198", strokeFor("inner"), "6 5")}
            ${signalLabel(612, 284, "inner feedback", strokeFor("inner"))}
            ${feedbackPath("M 786 346 C 786 430, 270 430, 270 198", strokeFor("outer"), "7 5")}
            ${signalLabel(422, 418, "measured output y feeds back to the outer summing junction", strokeFor("outer"))}
            ${collapseOverlay}

            <rect x="30" y="258" width="438" height="86" rx="14" fill="#ffffff" stroke="#dbe7ea"></rect>
            <text x="52" y="286" font-size="15" font-weight="700" fill="#172d33">Outer-loop design plant</text>
            <text x="52" y="314" font-size="15" fill="#57411d">P = P_out(s) · G_in,cl(s)</text>
            <text x="214" y="314" font-size="15" fill="#57411d">= P_out · P_in C_in / (1 + P_in C_in)</text>
            <text x="52" y="334" font-size="12" fill="#526047">Loop shaping uses the full closed inner-loop dynamics, not just the DC gain approximation.</text>

            <rect x="30" y="372" width="310" height="74" rx="14" fill="#ffffff" stroke="#dbe7ea"></rect>
            <text x="52" y="398" font-size="14" font-weight="700" fill="#172d33">Bandwidth separation meter</text>
            <rect x="52" y="416" width="240" height="14" rx="7" fill="#eef2f2"></rect>
            <rect x="52" y="416" width="${sepWidth}" height="14" rx="7" fill="${sepColor}"></rect>
            <text x="304" y="428" font-size="13" fill="${sepColor}">${fmt(separation, 0)}x</text>
            <text x="52" y="444" font-size="12" fill="#526047">${good ? "good: inner loop looks fast to the outer loop" : okay ? "usable but marginal: keep checking interaction" : "poor: outer loop sees too much inner-loop dynamics"}</text>

            <rect x="372" y="372" width="498" height="74" rx="14" fill="#ffffff" stroke="#dbe7ea"></rect>
            <text x="394" y="398" font-size="14" font-weight="700" fill="#172d33">Signal tracing takeaway</text>
            <text x="394" y="422" font-size="13" fill="#526047">The outer controller commands the input to the inner loop. After the fast inner loop is closed, the outer-loop designer treats that closed-loop response as part of the plant.</text>
          `;
        } else {
          drawAutoVisual(activity, "Successive loop closure map", "Trace signals and collapse the closed inner loop before designing the outer loop.", "", "0 0 900 520");
        }
      }
    };

      for (const activity of document.querySelectorAll("[data-advanced-activity]")) {
        try {
        normalizeAdvancedActivityLayout(activity);
        const type = activity.dataset.advancedActivity;
      const update = () => {
	        if (type === "first-order-step") {
	          const p = val(activity, "p", 1);
	          const a1 = val(activity, "a1", 1);
	          setText(activity, "p-value", fmt(p, 2));
	          setText(activity, "rise-time", fmt(Math.log(10) / p, 3));
	          setText(activity, "slope-a1", fmt(a1 * p, 3));
	          setHTML(activity, "p-display", `<span class="inline-equation">p</span> = ${fmt(p, 2)}`);
	          setHTML(activity, "rise-time-display", `<span class="inline-equation">t<sub>r,90</sub></span> = ${fmt(Math.log(10) / p, 3)} s`);
	          setHTML(activity, "slope-display", `<span class="inline-equation">A p</span> = ${fmt(a1 * p, 3)}`);
	          drawFirstOrderStepCanvas(activity, p, a1);
	        }
        if (type === "dirty-derivative") {
          const sigma = val(activity, "dirty-sigma", 0.06);
          const ts = 0.02;
          const beta = (2 * sigma - ts) / (2 * sigma + ts);
          setText(activity, "dirty-sigma", fmt(sigma, 3));
          setText(activity, "dirty-bandwidth", fmt(1 / sigma, 2));
          setText(activity, "dirty-beta", fmt(beta, 3));
          setText(
            activity,
            "dirty-note",
            sigma < 0.05
              ? "Small sigma follows rapid changes but passes more noise into the derivative estimate."
              : sigma > 0.35
                ? "Large sigma rejects noise strongly, but the derivative estimate lags the changing signal."
                : "Moderate sigma reduces the noise spikes while retaining much of the derivative trend."
          );
          drawDirtyDerivativeCanvas(activity, sigma);
        }
	        if (type === "digital-pid") {
	          const ts = val(activity, "ts", 0.05);
	          const e = val(activity, "error", 1);
          const ePrev = val(activity, "prev-error", 0.6);
          const sigma = val(activity, "sigma", 0.05);
          const trap = (ts / 2) * (e + ePrev);
          const diff = (e - ePrev) / ts;
          const beta = (2 * sigma - ts) / (2 * sigma + ts);
          setText(activity, "trap", fmt(trap, 4));
          setText(activity, "diff", fmt(diff, 3));
          setText(activity, "beta", fmt(beta, 3));
	          setText(activity, "sample-note", ts > 0.15 ? "coarse sample: discrete terms lag the signal" : "fine sample: discrete terms track the signal more closely");
	          drawActivityVisual(activity, type, { ts, e, ePrev, trap, diff });
	        }
        if (type === "anti-windup") {
          const enabled = Boolean(activity.querySelector("[data-input='aw-enabled']")?.checked);
          const method = selectValue(activity, "aw-method", "conditional");
          const limit = val(activity, "aw-limit", 0.75);
          const ki = val(activity, "aw-ki", 3);
          const baseline = simulateAntiWindup({ enabled: false, method: "none", limit, ki });
          const active = simulateAntiWindup({ enabled, method, limit, ki });
          const metrics = active.metrics;
          const methodLabel = enabled
            ? (method === "backcalc" ? "back-calculation correction" : "conditional integration")
            : "no anti-windup";
          setText(activity, "aw-limit-value", fmt(limit, 2));
          setText(activity, "aw-ki-value", fmt(ki, 2));
          setText(activity, "aw-overshoot", `${fmt(100 * metrics.overshoot, 1)}% (${methodLabel})`);
          setText(activity, "aw-saturation-time", `${fmt(metrics.saturationTime, 2)} s`);
          setText(activity, "aw-final-error", fmt(metrics.finalError, 3));
          setText(activity, "aw-peak-integrator", fmt(metrics.peakIntegrator, 3));
          drawAntiWindupCanvas(activity, active, baseline, limit);
        }
	        if (type === "inertia") {
	          const m = val(activity, "mass", 1);
          const r = val(activity, "radius", 0.5);
          const v = val(activity, "velocity", 1);
          const omega = val(activity, "omega", 2);
          const offset = val(activity, "offset", 0.2);
          const inertia = 2 * m * r * r;
          const shifted = inertia + 2 * m * offset * offset;
          const translational = 0.5 * (2 * m) * v * v;
          const rotational = 0.5 * shifted * omega * omega;
          const centerX = 120;
          const centerY = 82;
          const radiusPx = Math.min(80, Math.max(16, r * 36));
          const axisX = centerX + Math.min(60, offset * 38);
          const omegaR = 24;
          const omegaRight = axisX + omegaR;
          const omegaLeft = axisX - omegaR;
          setText(activity, "inertia", fmt(inertia, 3));
          setText(activity, "shifted", fmt(shifted, 3));
          setText(activity, "trans", fmt(translational, 3));
          setText(activity, "rot", fmt(rotational, 3));
          setText(activity, "mass-value", fmt(m, 2));
          setText(activity, "radius-value", fmt(r, 2));
          setText(activity, "velocity-value", fmt(v, 2));
          setText(activity, "omega-value", fmt(omega, 2));
          setText(activity, "offset-value", fmt(offset, 2));
          setAttr(activity, "mass-left", "cx", centerX - radiusPx);
          setAttr(activity, "mass-right", "cx", centerX + radiusPx);
          setAttr(activity, "rod", "x1", centerX - radiusPx);
          setAttr(activity, "rod", "x2", centerX + radiusPx);
          setAttr(activity, "axis", "x1", axisX);
          setAttr(activity, "axis", "x2", axisX);
          setAttr(activity, "axis-label", "x", Math.min(axisX + 6, 205));
          setAttr(activity, "offset-line", "x1", centerX);
          setAttr(activity, "offset-line", "x2", axisX);
          setAttr(activity, "radius-line", "x2", centerX + radiusPx);
          setAttr(activity, "omega-arc", "d", `M ${omegaRight} ${centerY} A ${omegaR} ${omegaR} 0 1 0 ${omegaLeft} ${centerY} A ${omegaR} ${omegaR} 0 1 0 ${omegaRight} ${centerY}`);
          setAttr(activity, "omega-head", "d", `M ${omegaRight - 7} ${centerY + 4} L ${omegaRight} ${centerY - 10} L ${omegaRight + 7} ${centerY + 4} Z`);
          setAttr(activity, "omega-label", "x", Math.min(axisX + omegaR + 18, 260));
          setAttr(activity, "omega-label", "y", centerY - 10);
        }
        if (type === "lagrange") {
          const k = val(activity, "k", 4);
          const x = val(activity, "x", 0.5);
          const m = val(activity, "mass", 1);
          const v = val(activity, "velocity", 1.2);
          const spring = 0.5 * k * x * x;
          const kinetic = 0.5 * m * v * v;
          const lagrangian = kinetic - spring;
          const derivative = k * x;
          setText(activity, "spring", fmt(spring, 3));
          setText(activity, "kinetic", fmt(kinetic, 3));
          setText(activity, "lagrangian", fmt(lagrangian, 3));
          setText(activity, "partial", fmt(derivative, 3));
          setText(activity, "k-value", fmt(k, 2));
          setText(activity, "q-value", fmt(x, 2));
          setText(activity, "mass-value", fmt(m, 2));
          setText(activity, "qdot-value", fmt(v, 2));
        }
        if (type === "linearization") {
          const x0 = val(activity, "x0", 1);
          const u0 = val(activity, "u0", -1);
          const x = val(activity, "x", 1.2);
          const u = val(activity, "u", u0);
          const f = x0 * x0 + u0;
          const a = 2 * x0;
          const b = 1;
          const tangent = f + a * (x - x0) + b * (u - u0);
          const actual = x * x + u;
          setText(activity, "equilibrium", fmt(f, 3));
          setText(activity, "a", fmt(a, 3));
          setText(activity, "b", fmt(b, 3));
          setText(activity, "error", fmt(actual - tangent, 3));
          setText(activity, "x0-value", fmt(x0, 2));
          setText(activity, "u0-value", fmt(u0, 2));
          setText(activity, "x-value", fmt(x, 2));
          setText(activity, "u-value", fmt(u, 2));
          setText(activity, "tangent", fmt(tangent, 3));
          setText(activity, "actual", fmt(actual, 3));
          drawLinearizationCanvas(activity, x0, u0, x, u);
          drawLinearizationErrorCanvas(activity, x0, u0, x, u);
        }
        if (type === "transfer-function") {
          const a1 = val(activity, "a1", 3);
          const a0 = val(activity, "a0", 2);
          const b0 = val(activity, "b0", 4);
          const disc = a1 * a1 - 4 * a0;
          const dc = a0 === 0 ? "undefined" : fmt(b0 / a0, 3);
          const polePoints = disc >= 0
            ? [
              { re: (-a1 + Math.sqrt(disc)) / 2, im: 0 },
              { re: (-a1 - Math.sqrt(disc)) / 2, im: 0 },
            ]
            : [
              { re: -a1 / 2, im: Math.sqrt(-disc) / 2 },
              { re: -a1 / 2, im: -Math.sqrt(-disc) / 2 },
            ];
          const poles = polePoints.map((pole) => pole.im === 0
            ? fmt(pole.re, 3)
            : `${fmt(pole.re, 3)} ${pole.im > 0 ? "+" : "-"} j${fmt(Math.abs(pole.im), 3)}`
          ).join(", ");
          setHTML(activity, "tf", `H(s) = <span class="inline-frac"><span class="frac-top">${fmt(b0)}</span><span class="frac-bottom">s<sup>2</sup> + ${fmt(a1)}s + ${fmt(a0)}</span></span>`);
          setText(activity, "poles", poles);
          setText(activity, "dc", dc);
          setText(activity, "order", "denominator order 2");
          drawActivityVisual(activity, type, { a1, a0, b0, dc, polePoints });
        }
        if (type === "state-space") {
          const a1 = val(activity, "a1", 3);
          const a0 = val(activity, "a0", 2);
          const b0 = val(activity, "b0", 1);
          const x1 = val(activity, "x1", 1);
          const x2 = val(activity, "x2", 0);
          const u = val(activity, "u", 1);
          const c1 = val(activity, "c1", 1);
          const c2 = val(activity, "c2", 0);
          const d = val(activity, "d", 0);
          const x1dot = x2;
          const x2dot = -a0 * x1 - a1 * x2 + b0 * u;
          const y = c1 * x1 + c2 * x2 + d * u;
          const term = (coefficient, variable, first = false) => {
            if (coefficient === 0) return first ? "0" : "";
            const sign = coefficient < 0 ? (first ? "-" : " - ") : (first ? "" : " + ");
            return `${sign}${fmt(Math.abs(coefficient), 2)}${variable}`;
          };
          const stateSecondEquation = `${term(-a0, "x<sub>1</sub>", true)}${term(-a1, "x<sub>2</sub>")}${term(b0, "u")}`;
          const outputEquation = `${term(c1, "x<sub>1</sub>", true)}${term(c2, "x<sub>2</sub>")}${term(d, "u")}`;
          setText(activity, "a1-value", fmt(a1, 2));
          setText(activity, "a0-value", fmt(a0, 2));
          setText(activity, "b0-value", fmt(b0, 2));
          setText(activity, "x1-value", fmt(x1, 2));
          setText(activity, "x2-value", fmt(x2, 2));
          setText(activity, "u-value", fmt(u, 2));
          setText(activity, "c1-value", fmt(c1, 2));
          setText(activity, "c2-value", fmt(c2, 2));
          setText(activity, "d-value", fmt(d, 2));
          setHTML(activity, "states", "x<sub>1</sub> = y, x<sub>2</sub> = y'");
          setHTML(activity, "stateeq", `<span class="overdot">x</span> = Ax + Bu, with x<sub>1</sub>' = x<sub>2</sub> and x<sub>2</sub>' = ${stateSecondEquation}`);
          setHTML(activity, "a", `A = [[0, 1], [${fmt(-a0, 2)}, ${fmt(-a1, 2)}]]`);
          setHTML(activity, "b", `B = [[0], [${fmt(b0, 2)}]]`);
          setHTML(activity, "xdot", `<span class="overdot">x</span> = [${fmt(x1dot, 3)}, ${fmt(x2dot, 3)}]`);
          setHTML(activity, "outputeq", `y = Cx + Du = ${outputEquation}`);
          setText(activity, "y", fmt(y, 3));
          drawActivityVisual(activity, type, { a1, a0, b0, x1, x2, u, c1, c2, d, x1dot, x2dot, y });
        }
        if (type === "state-feedback") {
          const p1 = val(activity, "p1", 2);
          const p2 = val(activity, "p2", 3);
          const a1 = p1 + p2;
          const a0 = p1 * p2;
          const k1 = a0 - 2;
          const k2 = a1 - 3;
          setHTML(activity, "poly", `s<sup>2</sup> + ${fmt(a1)}s + ${fmt(a0)}`);
          setHTML(activity, "gain", `[${fmt(k1)}, ${fmt(k2)}]`);
          setHTML(activity, "kr", fmt(a0));
          setText(activity, "poles", `-${fmt(p1)}, -${fmt(p2)}`);
          drawActivityVisual(activity, type, { p1, p2, poles: `-${fmt(p1)}, -${fmt(p2)}` });
        }
        if (type === "integrator-pole") {
          const pole = val(activity, "ipole", 4);
          const disturbance = val(activity, "disturbance", 0.2);
          const settling = 4 / pole;
          const effort = 1 + 0.35 * pole + 2 * disturbance;
          const overshoot = Math.max(0, 4 * pole - 8 + 8 * disturbance);
          setHTML(activity, "settling", `<span class="metric-equation">t<sub>s</sub> &approx; 4 / |p<sub>I</sub>| = 4 / ${fmt(pole)} = ${fmt(settling, 2)} s</span>`);
          setHTML(activity, "effort", `<span class="metric-equation">effort proxy = 1 + 0.35|p<sub>I</sub>| + 2d = ${fmt(effort, 2)}</span>`);
          setHTML(activity, "overshoot", `<span class="metric-equation">overshoot proxy = max(0, 4|p<sub>I</sub>| - 8 + 8d) = ${fmt(overshoot, 1)}%</span>`);
          setText(activity, "tradeoff", pole > 5 ? "fast integrator: better bias rejection but more control effort" : "slow integrator: gentler control but slower bias rejection");
        }
        if (type === "observer-innovation") {
          const l1 = val(activity, "l1", 3);
          const l2 = val(activity, "l2", 2);
          const disc = l1 * l1 - 4 * l2;
          const poles = disc >= 0
            ? [(-l1 + Math.sqrt(disc)) / 2, (-l1 - Math.sqrt(disc)) / 2].map((p) => fmt(p, 3)).join(", ")
            : `${fmt(-l1 / 2, 3)} +/- j${fmt(Math.sqrt(-disc) / 2, 3)}`;
          let x1 = 1;
          let x2 = 0.4;
          let h1 = -1;
          let h2 = -0.2;
          const dt = 0.02;
          for (let k = 0; k <= 12 / dt; k += 1) {
            const innovation = x1 - h1;
            const x1dot = x2;
            const h1dot = h2 + l1 * innovation;
            const h2dot = l2 * innovation;
            x1 += dt * x1dot;
            h1 += dt * h1dot;
            h2 += dt * h2dot;
            if (Math.abs(h1) > 1e4 || Math.abs(h2) > 1e4) break;
          }
          const finalError = Math.hypot(x1 - h1, x2 - h2);
          setHTML(activity, "innovation-l", `L = [${fmt(l1)}, ${fmt(l2)}]<sup>T</sup>`);
          setHTML(activity, "innovation-matrix", `A - LC = [[-${fmt(l1)}, 1], [-${fmt(l2)}, 0]]`);
          setText(activity, "innovation-poles", poles);
          setText(activity, "innovation-error", finalError > 999 ? "diverging" : fmt(finalError, 3));
          drawInnovationCanvas(activity, l1, l2);
        }
        if (type === "car-observability") {
          const c1 = val(activity, "c1", 1);
          const c2 = val(activity, "c2", 0);
          const det = c1 * c1;
          const rank = Math.abs(c1) > 1e-9 ? 2 : (Math.abs(c2) > 1e-9 ? 1 : 0);
          setHTML(activity, "car-c", `C = [${fmt(c1)}, ${fmt(c2)}]`);
          setHTML(activity, "car-o", `O = [[${fmt(c1)}, ${fmt(c2)}], [0, ${fmt(c1)}]]`);
          setText(activity, "car-det", fmt(det, 3));
          setText(activity, "car-rank", rank === 2 ? "rank 2: observable" : `rank ${rank}: not fully observable`);
          drawCarObservabilityVisual(activity, c1, c2);
        }
        if (type === "observer") {
          const speed = val(activity, "speed", 5);
          const noise = val(activity, "noise", 0.05);
          const decay = 4 / speed;
          const amplification = noise * speed;
          setText(activity, "opoles", `observer poles near -${fmt(speed)}`);
          setText(activity, "decay", `${fmt(decay, 2)} s`);
          setText(activity, "noise", fmt(amplification, 3));
          setText(activity, "observer-note", speed > 8 ? "fast convergence, but noise is amplified" : "slower convergence, but noise sensitivity is lower");
          drawActivityVisual(activity, type, { speed, noise: amplification });
        }
        if (type === "disturbance-observer") {
          const reference = val(activity, "reference", 1);
          const d = val(activity, "disturbance", 0.35);
          const pole = val(activity, "disturbance-pole", 2);
          const simulation = simulateCh14DisturbanceObserver({
            disturbance: d,
            poleMagnitude: pole,
            reference,
            mode: "augmented",
          });
          setText(activity, "dhat", fmt(simulation.metrics.finalDhat, 3));
          setText(activity, "tracking-error", fmt(simulation.metrics.finalError, 4));
          setText(
            activity,
            "dhat-settling",
            Number.isFinite(simulation.metrics.settlingTime) ? `${fmt(simulation.metrics.settlingTime, 2)} s after disturbance` : "not settled in plotted window",
          );
          setText(
            activity,
            "dist-note",
            pole > 5
              ? "fast disturbance pole: quicker estimate convergence with a sharper transient"
              : "slower disturbance pole: gentler estimate convergence with longer residual innovation",
          );
          drawCh14ObserverCanvas(
            activity,
            "disturbance-observer",
            [{ rows: simulation.rows, label: "augmented disturbance observer", color: "#0e6d77" }],
            { extraLegend: "dashed blue: reference r; dashed brown: true disturbance d" },
          );
        }
        if (type === "disturbance-observer-comparison") {
          const reference = val(activity, "reference", 1);
          const d = val(activity, "disturbance", 0.35);
          const pole = val(activity, "disturbance-pole", 2);
          const standard = simulateCh14DisturbanceObserver({
            disturbance: d,
            poleMagnitude: pole,
            reference,
            mode: "standard",
          });
          const augmented = simulateCh14DisturbanceObserver({
            disturbance: d,
            poleMagnitude: pole,
            reference,
            mode: "augmented",
          });
          setText(activity, "standard-error", fmt(standard.metrics.finalError, 4));
          setText(activity, "augmented-error", fmt(augmented.metrics.finalError, 4));
          setText(activity, "comparison-dhat", fmt(augmented.metrics.finalDhat, 3));
          setText(
            activity,
            "comparison-note",
            Math.abs(augmented.metrics.finalError) < Math.abs(standard.metrics.finalError)
              ? "the augmented observer removes the matched constant-disturbance bias"
              : "with this setting the augmented transient has not yet improved the final plotted error",
          );
          drawCh14ObserverCanvas(
            activity,
            "disturbance-observer-comparison",
            [
              { rows: standard.rows, label: "standard observer", color: "#8a99a0", dash: [7, 5], width: 2.3 },
              { rows: augmented.rows, label: "augmented disturbance observer", color: "#0e6d77", width: 2.8 },
            ],
            {
              panels: [
                { title: "Output y(t)", value: (row) => row.y, guides: [{ value: (row) => row.r, color: "#5c6bc0", dash: [6, 5] }] },
                { title: "Tracking error r - y", value: (row) => row.error },
                { title: "Disturbance estimate", value: (row) => row.dhat, guides: [{ value: (row) => row.trueD, color: "#8b5e24", dash: [6, 5] }] },
                { title: "Innovation y - C x_hat", value: (row) => row.innovation },
              ],
              extraLegend: "dashed blue: reference r; dashed brown: true disturbance d",
            },
          );
        }
        if (type === "disturbance-code-map") {
          const answers = {
            "map-a2": "a2",
            "map-c2": "c2",
            "map-innovation": "innovation",
            "map-extract": "extract",
            "map-control": "minus-dhat",
          };
          const entries = Object.entries(answers);
          const selected = entries.map(([key, answer]) => selectValue(activity, key, "") === answer);
          const completed = entries.filter(([key]) => selectValue(activity, key, "")).length;
          const correct = selected.filter(Boolean).length;
          setText(activity, "code-map-score", `${correct} / ${entries.length}`);
          setText(
            activity,
            "code-map-note",
            correct === entries.length
              ? "all code lines are matched to the disturbance-observer equations"
              : completed === 0
                ? "choose a role for each code line"
                : "focus on the innovation term and the final -dhat compensation; those are the two ideas that distinguish the disturbance observer",
          );
        }
        if (type === "disturbance-observability") {
          const outputKey = selectValue(activity, "dist-output", "position");
          const channelKey = selectValue(activity, "dist-channel", "input");
          const result = ch14DisturbanceObservability(outputKey, channelKey);
          const matrixHtml = result.o
            .map((row) => `[${row.map((value) => fmt(value, 2)).join(", ")}]`)
            .join("<br>");
          setHTML(activity, "dist-o-matrix", `<span class="metric-equation">${matrixHtml}</span>`);
          setText(activity, "dist-o-rank", `${result.rank} / 3`);
          setText(
            activity,
            "dist-o-note",
            result.rank === 3
              ? "observable: the output history can reveal both plant states and the constant disturbance"
              : "not fully observable: this sensor/channel pairing hides part of the augmented state",
          );
          drawCh14DisturbanceObservabilityVisual(activity, result);
        }
        if (type === "pd-poles") {
          const sigma = val(activity, "sigma", 2);
          const wd = val(activity, "wd", 2);
          const a1 = val(activity, "a1", 1);
          const a0 = val(activity, "a0", 0);
          const b0 = val(activity, "b0", 1);
          const desiredA1 = 2 * sigma;
          const desiredA0 = sigma * sigma + wd * wd;
          const kd = (desiredA1 - a1) / b0;
          const kp = (desiredA0 - a0) / b0;
          const wn = Math.sqrt(desiredA0);
          const zeta = sigma / wn;
          setText(activity, "sigma-value", fmt(sigma, 2));
          setText(activity, "wd-value", fmt(wd, 2));
          setText(activity, "a1-value", fmt(a1, 2));
          setText(activity, "a0-value", fmt(a0, 2));
          setText(activity, "b0-value", fmt(b0, 2));
          setHTML(activity, "plant", `s<sup>2</sup> + ${fmt(a1, 3)}s + ${fmt(a0, 3)}`);
          setHTML(activity, "poly", `s<sup>2</sup> + ${fmt(desiredA1, 3)}s + ${fmt(desiredA0, 3)}`);
          setHTML(activity, "gains", `k<sub>P</sub> = ${fmt(kp, 3)}, k<sub>D</sub> = ${fmt(kd, 3)}`);
          setHTML(activity, "coefficient-check", `a<sub>1</sub> + b<sub>0</sub>k<sub>D</sub> = ${fmt(a1, 3)} + ${fmt(b0, 3)}(${fmt(kd, 3)}) = ${fmt(a1 + b0 * kd, 3)}; a<sub>0</sub> + b<sub>0</sub>k<sub>P</sub> = ${fmt(a0, 3)} + ${fmt(b0, 3)}(${fmt(kp, 3)}) = ${fmt(a0 + b0 * kp, 3)}`);
          setText(activity, "zeta", fmt(zeta, 3));
          setText(activity, "wn", fmt(wn, 3));
          drawActivityVisual(activity, type, { sigma, wd, poles: `-${fmt(sigma)} +/- j${fmt(wd)}` });
        }
        if (type === "ch15-sine-response") {
          const omega = val(activity, "omega", 1);
          const mag = 1 / Math.sqrt(1 + omega * omega);
          const phase = -Math.atan(omega) * 180 / Math.PI;
          setText(activity, "sine-omega", fmt(omega, 2));
          setText(activity, "sine-mag", fmt(mag, 3));
          setText(activity, "sine-phase", `${fmt(phase, 1)} deg`);
          setText(activity, "sine-note", omega < 0.5 ? "low frequency: little attenuation or lag" : omega > 3 ? "high frequency: strong attenuation and large lag" : "mid frequency: attenuation and lag are both visible");
          drawCh15SineResponse(activity, omega);
        }
        if (type === "ch15-frequency-scale") {
          const omegaExp = val(activity, "omega-exp", 0);
          const omega = 10 ** omegaExp;
          const mag = 1 / Math.sqrt(1 + omega * omega);
          const db = 20 * Math.log10(mag);
          const phase = -Math.atan(omega) * 180 / Math.PI;
          const region = omega < 0.5 ? "below the corner frequency" : omega > 2 ? "above the corner frequency" : "near the corner frequency";
          const note = omega < 0.5
            ? "Low frequency: the magnitude is close to 1 and the phase lag is small."
            : omega > 2
              ? "High frequency: the magnitude rolls off and the phase approaches -90 deg."
              : "Corner region: attenuation and phase lag are both changing rapidly.";
          setText(activity, "scale-exp", fmt(omegaExp, 2));
          setText(activity, "scale-omega", fmt(omega, 3));
          setText(activity, "scale-location", `${fmt(omega, 3)} rad/s, ${region}`);
          setText(activity, "scale-mag", fmt(mag, 3));
          setText(activity, "scale-db", `20log10(${fmt(mag, 3)}) = ${fmt(db, 2)} dB`);
          setText(activity, "scale-phase", `-atan(${fmt(omega, 3)}) = ${fmt(phase, 1)} deg`);
          setText(activity, "scale-note", note);
          drawCh15FrequencyScale(activity, omega);
        }
        if (type === "ch15-pole-zero-builder") {
          const element = selectValue(activity, "element", "zero-lhp");
          const corner = val(activity, "corner", 1);
          const isZero = element.startsWith("zero");
          const isRhp = element.endsWith("rhp");
          const cornerResponse = poleZeroResponse(element, corner, corner);
          const wc = fmt(corner, 2);
          const numerator = isZero
            ? (isRhp ? `${wc} - s` : `s + ${wc}`)
            : wc;
          const denominator = isZero
            ? wc
            : (isRhp ? `${wc} - s` : `s + ${wc}`);
          const factorText = `<span class="metric-equation ch15-pz-factor-equation"><span class="inline-equation">H(s)</span> = <span class="inline-frac"><span class="frac-top">${numerator}</span><span class="frac-bottom">${denominator}</span></span></span>`;
          const phaseStart = isZero && isRhp ? 180 : 0;
          const phaseEnd = isZero ? 90 : (isRhp ? 90 : -90);
          const formatDegrees = (degrees) => `${Math.round(degrees)}`;
          setHTML(activity, "pz-factor", factorText);
          setText(activity, "pz-corner-frequency", fmt(corner, 2));
          setHTML(
            activity,
            "pz-slope",
            `<span class="metric-equation">${isZero ? "+20" : "-20"} dB/dec for <span class="inline-equation">&omega; &gt; &omega;<sub>c</sub></span></span>`,
          );
          setHTML(
            activity,
            "pz-phase-change",
            `<span class="metric-equation">${formatDegrees(phaseStart)} deg &rarr; ${formatDegrees(phaseEnd)} deg; exact phase at <span class="inline-equation">&omega;<sub>c</sub></span> = ${fmt(cornerResponse.phase, 1)} deg</span>`,
          );
          setHTML(
            activity,
            "pz-corner",
            `<span class="metric-equation">exact magnitude at <span class="inline-equation">&omega;<sub>c</sub></span> = ${fmt(cornerResponse.magDb, 2)} dB</span>`,
          );
          setHTML(
            activity,
            "pz-guides",
            `<span class="metric-equation">phase transition guide: ${fmt(corner / 10, 3)} to ${fmt(corner * 10, 3)} rad/s</span>`,
          );
          drawCh15PoleZeroBuilder(activity, element, corner);
        }
        if (type === "ch15-second-order-bode") {
          const zeta = val(activity, "zeta", 0.7);
          const wn = val(activity, "wn", 1);
          const w = logspace(-2, 2, 420);
          const mags = w.map((x) => {
            const r = x / wn;
            return 20 * Math.log10(1 / Math.hypot(1 - r * r, 2 * zeta * r));
          });
          const peak = Math.max(...mags);
          setText(activity, "so-zeta", fmt(zeta, 2));
          setText(activity, "so-wn", fmt(wn, 2));
          setText(activity, "so-peak", `${fmt(peak, 2)} dB`);
          setText(activity, "so-phase-wn", "-90 deg");
          setText(activity, "so-slope", "-40 dB/dec");
          drawCh15SecondOrderBode(activity, zeta, wn);
        }
        if (type === "ch15-example-1-builder") {
          const omegaExp = val(activity, "ex1-omega", 1);
          const slopePrediction = selectValue(activity, "ex1-slope-predict", "");
          const phasePrediction = selectValue(activity, "ex1-phase-predict", "");
          const checked = activity.dataset.ex1PredictionChecked === "true";
          const figure = activity.querySelector(".ch15-example1-figure");
          const readout = activity.querySelector(".ch15-example1-readout");
          const feedback = activity.querySelector('[data-output="ex1-prediction"]');
          const ready = Boolean(slopePrediction && phasePrediction);
          setText(activity, "ex1-omega-exp", fmt(omegaExp, 2));
          setText(activity, "ex1-omega", fmt(10 ** omegaExp, 3));
          if (!checked) {
            if (figure) figure.hidden = true;
            if (readout) readout.hidden = true;
            feedback?.classList.remove("is-correct", "is-wrong");
            setText(activity, "ex1-prediction", ready ? "Click Check predictions to reveal the plot." : "Choose both predictions before checking.");
          } else if (!ready) {
            if (figure) figure.hidden = true;
            if (readout) readout.hidden = true;
            feedback?.classList.remove("is-correct", "is-wrong");
            setText(activity, "ex1-prediction", "Choose both predictions before checking.");
          } else {
            if (figure) figure.hidden = false;
            if (readout) readout.hidden = false;
            const result = drawCh15Example1Builder(activity, omegaExp);
            const activeLabels = result.active.map((term) => term.label);
            const magError = result.approxProbe.magDb - result.exactProbe.magDb;
            const phaseError = result.approxProbe.phase - result.exactProbe.phase;
            const slopeCorrect = slopePrediction === "-40";
            const phaseCorrect = phasePrediction === "-180";
            const slopeFeedback = slopeCorrect ? "slope correct" : "slope should be -40 dB/dec after the pole is included";
            const phaseFeedback = phaseCorrect ? "phase correct" : "high-frequency phase tends to -180 deg";
            feedback?.classList.toggle("is-correct", slopeCorrect && phaseCorrect);
            feedback?.classList.toggle("is-wrong", !(slopeCorrect && phaseCorrect));
            setText(activity, "ex1-active-terms", activeLabels.length ? activeLabels.join(", ") : "none");
            setText(activity, "ex1-lesson", describeExample1Lesson(result.active));
            setText(activity, "ex1-probe-mag", `exact ${fmt(result.exactProbe.magDb, 2)} dB; approximation ${fmt(result.approxProbe.magDb, 2)} dB; error ${fmt(magError, 2)} dB`);
            setText(activity, "ex1-probe-phase", `exact ${fmt(result.exactProbe.phase, 1)} deg; approximation ${fmt(result.approxProbe.phase, 1)} deg; error ${fmt(phaseError, 1)} deg`);
            setText(activity, "ex1-prediction", `${slopeFeedback}; ${phaseFeedback}`);
          }
        }
        if (type === "ch15-bode-example-builder") {
          const exampleKey = selectValue(activity, "example", "example1");
          const terms = ch15ExampleTerms[exampleKey] || ch15ExampleTerms.example1;
          updateCh15ExampleTermCheckboxes(activity, terms, exampleKey);
          const omegaExp = val(activity, "example-omega", 1);
          const slopePrediction = selectValue(activity, "example-slope-predict", "");
          const phasePrediction = selectValue(activity, "example-phase-predict", "");
          const checked = activity.dataset.examplePredictionChecked === "true";
          const figure = activity.querySelector(".ch15-example-builder-figure");
          const readout = activity.querySelector(".ch15-example-builder-readout");
          const feedback = activity.querySelector('[data-output="example-prediction"]');
          const ready = Boolean(slopePrediction && phasePrediction);
          setText(activity, "example-omega-exp", fmt(omegaExp, 2));
          setText(activity, "example-omega", fmt(10 ** omegaExp, 3));
          if (!checked) {
            if (figure) figure.hidden = true;
            if (readout) readout.hidden = true;
            feedback?.classList.remove("is-correct", "is-wrong");
            setText(activity, "example-prediction", ready ? "Click Check predictions to reveal the construction plot." : "Choose both predictions before checking.");
          } else if (!ready) {
            if (figure) figure.hidden = true;
            if (readout) readout.hidden = true;
            feedback?.classList.remove("is-correct", "is-wrong");
            setText(activity, "example-prediction", "Choose both predictions before checking.");
          } else {
            if (figure) figure.hidden = false;
            if (readout) readout.hidden = false;
            const result = drawCh15BodeExampleBuilder(activity, exampleKey, omegaExp);
            const expected = exampleFinalBehavior(result.terms);
            const magError = result.approxProbe.magDb - result.exactProbe.magDb;
            const phaseError = result.approxProbe.phase - result.exactProbe.phase;
            const slopeCorrect = Number(slopePrediction) === expected.slope;
            const phaseCorrect = Number(phasePrediction) === expected.phase;
            const slopeFeedback = slopeCorrect ? "slope correct" : `slope should be ${expected.slope} dB/dec after all factors are included`;
            const phaseFeedback = phaseCorrect ? "phase correct" : `high-frequency phase tends to ${expected.phase} deg`;
            feedback?.classList.toggle("is-correct", slopeCorrect && phaseCorrect);
            feedback?.classList.toggle("is-wrong", !(slopeCorrect && phaseCorrect));
            setText(activity, "example-terms", `${result.included.length} of ${result.total}: ${result.included.length ? result.included.map((term) => term.label).join(", ") : "none selected"}`);
            setText(activity, "example-lesson", describeExampleBuilderLesson(result.included, result.terms));
            setText(activity, "example-probe-mag", `exact ${fmt(result.exactProbe.magDb, 2)} dB; approximation ${fmt(result.approxProbe.magDb, 2)} dB; error ${fmt(magError, 2)} dB`);
            setText(activity, "example-probe-phase", `exact ${fmt(result.exactProbe.phase, 1)} deg; approximation ${fmt(result.approxProbe.phase, 1)} deg; error ${fmt(phaseError, 1)} deg`);
            setText(activity, "example-prediction", `${slopeFeedback}; ${phaseFeedback}`);
          }
        }
        if (type === "ch18-block-explorer") {
          const block = selectValue(activity, "ch18-block-type", "lead");
          const cornerExp = val(activity, "ch18-block-corner", 0);
          const corner = 10 ** cornerExp;
          const m = val(activity, "ch18-block-m", 10);
          const probeExp = val(activity, "ch18-block-probe", 0);
          const result = drawCh18BlockExplorer(activity, block, cornerExp, m, probeExp);
          const info = result.details;
          const mControl = activity.querySelector(".ch18-m-control");
          if (mControl) mControl.hidden = !["lag", "lead"].includes(block);
          setHTML(activity, "ch18-block-corner-label", info.cornerLabel);
          setHTML(activity, "ch18-block-corner-value", info.cornerValue);
          setHTML(activity, "ch18-block-ratio-label", info.ratioLabel || "Pole-zero ratio");
          setHTML(activity, "ch18-block-m-value", info.mValue);
          setText(activity, "ch18-block-probe-value", fmt(result.probe, 3));
          setHTML(activity, "ch18-block-transfer", `<span class="metric-equation">${info.transfer}</span>`);
          setText(activity, "ch18-block-derived", info.derived);
          setText(activity, "ch18-block-hint", info.hint);
          setText(activity, "ch18-block-mag-effect", info.magEffect);
          setText(activity, "ch18-block-phase-effect", info.phaseEffect);
          setText(activity, "ch18-block-use", info.use);
          setText(activity, "ch18-block-warning", info.warning);
          setHTML(activity, "ch18-block-probe-readout", `At the frequency marker <span class="inline-equation">&omega;</span> = ${fmt(result.probe, 3)} rad/s: |C(j<span class="inline-equation">&omega;</span>)| = ${fmt(result.probeResponse.magDb, 2)} dB, phase = ${fmt(result.probeResponse.phase, 1)} deg; ${info.regionText}.`);
        }
        if (type === "ch18-loopshape-sequence") {
          const step = Math.max(0, Math.min(4, Number.parseInt(activity.dataset.ch18SequenceStep || "0", 10)));
          const info = ch18LoopshapeSteps[step] || ch18LoopshapeSteps[0];
          for (const button of activity.querySelectorAll("[data-ch18-sequence-step]")) {
            button.classList.toggle("is-active", Number.parseInt(button.dataset.ch18SequenceStep || "0", 10) === step);
          }
          const margin = ch18FindGainCrossover(step);
          const phaseText = Number.isFinite(margin.phaseMargin)
            ? `gain crossover ${fmt(margin.wc, 2)} rad/s; PM ≈ ${fmt(margin.phaseMargin, 1)} deg`
            : "no gain crossover in the plotted range";
          const pill = (label, state) => `<span class="ch18-status-pill is-${state}">${state === "pass" ? "pass" : state === "warn" ? "watch" : "fail"}: ${label}</span>`;
          const statusHtml = `<span class="ch18-status-list">${
            [
              pill("tracking", step >= 3 ? "pass" : step >= 2 ? "warn" : "fail"),
              pill("disturbance", step >= 3 ? "pass" : "fail"),
              pill("noise", step >= 4 ? "pass" : "fail"),
              pill("phase margin", Number.isFinite(margin.phaseMargin) && margin.phaseMargin >= 45 ? "pass" : step === 0 ? "warn" : "fail"),
            ].join("")
          }</span>`;
          setHTML(activity, "ch18-sequence-added", `${info.added}: <span class="inline-equation">${info.block}</span>`);
          setHTML(activity, "ch18-sequence-controller", `<span class="inline-equation">${info.controller}</span>`);
          setText(activity, "ch18-sequence-change", info.change);
          setText(activity, "ch18-sequence-prompt", info.prompt);
          setHTML(activity, "ch18-sequence-status", statusHtml);
          setText(activity, "ch18-sequence-margin", phaseText);
          drawCh18LoopshapeSequence(activity, step);
        }
        if (type === "ch18-prefilter") {
          const p = val(activity, "ch18-prefilter-p", 9);
          setText(activity, "ch18-prefilter-p-value", fmt(p, 1));
          setText(activity, "ch18-prefilter-form", `F(s) = ${fmt(p, 2)} / (s + ${fmt(p, 2)})`);
          setText(activity, "ch18-prefilter-peak", p < 7 ? "strong peak reduction, slower response" : p > 18 ? "weak peak reduction, faster response" : "moderate peak reduction");
          setText(activity, "ch18-prefilter-note", p < 7 ? "less ringing but more command delay" : "faster command following but more resonant content passes");
          drawCh18Prefilter(activity, p);
        }
        if (type === "ch18-slc") {
          const step = selectValue(activity, "ch18-slc-step", "inner");
          const separation = val(activity, "ch18-slc-separation", 10);
          const check = selectValue(activity, "ch18-slc-check", "");
          for (const button of activity.querySelectorAll("[data-ch18-slc-step]")) {
            button.classList.toggle("is-active", button.dataset.ch18SlcStep === step);
          }
          const focus = {
            inner: "Step 1: design the fast inner loop using C_in(s) and P_in(s).",
            collapse: "Step 2: close the inner loop and replace it by an equivalent closed-loop block.",
            outer: "Step 3: design C_out(s) using P_out(s) cascaded with the closed inner-loop dynamics.",
            prefilter: "Step 4: add F(s) on the reference path to shape commands without changing disturbance rejection.",
          };
          const plant = {
            inner: "P_in(s)",
            collapse: "G_in,cl(s) = P_in(s)C_in(s)/(1 + P_in(s)C_in(s))",
            outer: "P_out(s)G_in,cl(s)",
            prefilter: "F(s) sits before the outer loop; the feedback plant is still P_out(s)G_in,cl(s)",
          };
          const equation = {
            inner: "L_in(s) = P_in(s)C_in(s)",
            collapse: "G_in,cl(s) = P_in(s)C_in(s)/(1 + P_in(s)C_in(s))",
            outer: "P(s) = P_out(s)G_in,cl(s)",
            prefilter: "r_f(s) = F(s)r(s)",
          };
          const why = {
            inner: "The inner loop must be the fastest because the outer loop will command it as an actuator-like subsystem.",
            collapse: "Collapsing the inner loop makes clear what dynamics the outer-loop controller actually sees.",
            outer: "Loop-shaping design keeps the full closed inner-loop dynamics instead of replacing them with only DC gain.",
            prefilter: "The prefilter changes command following, but it is outside the feedback loop and does not fix stability margins.",
          };
          const separationStatus = separation >= 8 ? "good separation" : separation >= 5 ? "marginal separation" : "too little separation";
          const feedback = check
            ? (check === "closed" ? "correct: use P_out times the closed inner-loop dynamics" : "not quite: the outer-loop plant is P_out times the closed inner-loop dynamics")
            : "choose an answer";
          setText(activity, "ch18-slc-separation-value", `${fmt(separation, 0)}x`);
          setText(activity, "ch18-slc-separation-status", separationStatus);
          setText(activity, "ch18-slc-focus", focus[step] || focus.inner);
          setHTML(activity, "ch18-slc-plant", `<span class="inline-equation">${plant[step] || plant.inner}</span>`);
          setHTML(activity, "ch18-slc-equation", `<span class="inline-equation">${equation[step] || equation.inner}</span>`);
          setText(activity, "ch18-slc-why", why[step] || why.inner);
          setText(activity, "ch18-slc-check-feedback", feedback);
          drawActivityVisual(activity, type, { step, separation });
        }
        if (type === "complex-response") {
          const real = val(activity, "real", 1);
          const imag = val(activity, "imag", 1);
          const mag = Math.hypot(real, imag);
          const phase = Math.atan2(imag, real) * 180 / Math.PI;
          const db = 20 * Math.log10(Math.max(mag, 1e-9));
          setText(activity, "rect", `${fmt(real)} + j${fmt(imag)}`);
          setText(activity, "mag", fmt(mag, 3));
          setText(activity, "phase", `${fmt(phase, 1)} deg`);
          setText(activity, "db", `${fmt(db, 1)} dB`);
          drawActivityVisual(activity, type, { real, imag, mag, phase });
        }
        if (type === "signal-path") {
          const sourceKey = selectValue(activity, "signal-path-source", "reference");
          const definition = signalPathDefinitions[sourceKey] || signalPathDefinitions.reference;
          setHTML(activity, "signal-transfer", definition.transfer);
          setText(activity, "signal-note", definition.note);
          drawSignalPathCanvas(activity, sourceKey);
        }
        if (type === "system-type") {
          const exampleKey = selectValue(activity, "system-type-example", "unknown-a");
          const example = systemTypeExamples[exampleKey] || systemTypeExamples["unknown-a"];
          const actualType = example.type;
          const inputOrder = Number.parseInt(selectValue(activity, "system-type-input", "0"), 10);
          const guessText = selectValue(activity, "system-type-guess", "");
          const guessedType = guessText === "" ? Number.NaN : Number.parseInt(guessText, 10);
          let logW1 = val(activity, "system-type-w1", -2.5);
          let logW2 = val(activity, "system-type-w2", -1.5);
          const amplitude = val(activity, "system-type-amplitude", 1);
          if (Math.abs(logW2 - logW1) < 0.08) logW2 = logW1 + 0.08;
          const db1 = systemTypeLoopDbAt(example, logW1);
          const db2 = systemTypeLoopDbAt(example, logW2);
          const slope = (db2 - db1) / (logW2 - logW1);
          const nearestMeasuredType = Math.max(0, Math.min(2, Math.round(Math.abs(slope) / 20)));
          const classificationChecked = activity.dataset.systemTypeChecked === "true";
          const classificationCorrect = classificationChecked && guessedType === actualType;
          const guideDb1 = -20 * inputOrder * logW1;
          const guideDb2 = -20 * inputOrder * logW2;
          const gapDb = ((db1 - guideDb1) + (db2 - guideDb2)) / 2;
          const constant = 10 ** (gapDb / 20);
          const inputLabels = ["step", "ramp", "parabolic"];
          const constantNames = ["M_p", "M_v", "M_a"];
          const constantHtml = ["M<sub>p</sub>", "M<sub>v</sub>", "M<sub>a</sub>"];
          const constantLabels = [`position constant ${constantHtml[0]}`, `velocity constant ${constantHtml[1]}`, `acceleration constant ${constantHtml[2]}`];
          const feedback = activity.querySelector('[data-output="system-type-check-feedback"]');
          let errorText;
          let note;
          let classificationText;
          let feedbackText;
          let feedbackOk = false;

          if (!classificationChecked) {
            classificationText = "Not checked yet.";
            feedbackText = "Measure the slope, choose a type, and click Check classification.";
            errorText = "Classify the system type first.";
            note = "The relevant guide and error constant are shown after a correct type classification.";
          } else if (!Number.isFinite(guessedType)) {
            classificationText = "No type selected.";
            feedbackText = "Choose Type 0, Type 1, or Type 2 before checking.";
            errorText = "Classify the system type first.";
            note = "Use the measured low-frequency slope: 0, -20, or -40 dB/decade.";
          } else if (guessedType !== actualType) {
            classificationText = `Not yet. The measured slope is closest to Type ${nearestMeasuredType}.`;
            feedbackText = "Not quite. Use the low-frequency trend: each free integrator contributes -20 dB/decade.";
            errorText = "Classify the system type correctly first.";
            note = "If the markers are near a bend or noisy region, move them lower and roughly one decade apart.";
          } else {
            feedbackOk = true;
            classificationText = `Correct: Type ${actualType}.`;
            feedbackText = `Correct. ${example.note}`;
            if (actualType < inputOrder) {
              errorText = `<span class="metric-equation">e<sub>ss</sub> &rarr; &infin;</span>`;
              note = `Type ${actualType} does not have enough free integrators for a ${inputLabels[inputOrder]} input.`;
            } else if (actualType > inputOrder) {
              errorText = `<span class="metric-equation">e<sub>ss</sub> &rarr; 0 as &omega; &rarr; 0</span>`;
              note = `Type ${actualType} has more free integrators than required for a ${inputLabels[inputOrder]} input.`;
            } else if (inputOrder === 0) {
              const error = amplitude / (1 + constant);
              errorText = `<span class="metric-equation">e<sub>ss</sub> = A / (1 + M<sub>p</sub>) = ${fmt(error, 4)}</span>`;
              note = "For Type 0 step tracking, the finite error is set by the position constant.";
            } else {
              const error = amplitude / constant;
              errorText = `<span class="metric-equation">e<sub>ss</sub> = A / ${constantHtml[inputOrder]} = ${fmt(error, 4)}</span>`;
              note = `For Type ${actualType} ${inputLabels[inputOrder]} tracking, the finite error is set by ${constantHtml[inputOrder]}.`;
            }
          }

          if (feedback) {
            feedback.classList.toggle("is-correct", feedbackOk);
            feedback.classList.toggle("is-wrong", classificationChecked && !feedbackOk);
          }
          setText(activity, "system-type-w1-exp", fmt(logW1, 2));
          setText(activity, "system-type-w2-exp", fmt(logW2, 2));
          setText(activity, "system-type-w1", fmt(10 ** logW1, 4));
          setText(activity, "system-type-w2", fmt(10 ** logW2, 4));
          setText(activity, "system-type-db1", fmt(db1, 2));
          setText(activity, "system-type-db2", fmt(db2, 2));
          setText(activity, "system-type-amplitude", fmt(amplitude, 2));
          setText(activity, "system-type-check-feedback", feedbackText);
          setHTML(
            activity,
            "system-type-slope",
            `<span class="metric-equation">slope = ${fmt(slope, 2)} dB/decade from <span class="inline-equation">&omega;<sub>1</sub></span> and <span class="inline-equation">&omega;<sub>2</sub></span></span>`,
          );
          setHTML(activity, "system-type-classification", classificationText);
          setHTML(
            activity,
            "system-type-constant",
            classificationCorrect
              ? `<span class="metric-equation">${constantLabels[inputOrder]}: gap = ${fmt(gapDb, 2)} dB, so ${constantHtml[inputOrder]} = 10<sup>${fmt(gapDb, 2)} / 20</sup> = ${fmt(constant, 4)}</span>`
              : "Shown after a correct type classification.",
          );
          setHTML(activity, "system-type-error", errorText);
          setHTML(activity, "system-type-note", note);
          drawSystemTypeCanvas(activity, {
            example,
            exampleLabel: example.label,
            actualType,
            guessedType,
            classificationCorrect,
            inputOrder,
            inputLabel: inputLabels[inputOrder],
            logW1,
            logW2,
            db1,
            db2,
            slope,
            amplitude,
            gapDb,
          });
        }
        if (type === "frequency-spec") {
          const specKey = selectValue(activity, "frequency-spec-kind", "tracking");
          const plantKey = selectValue(activity, "frequency-spec-plant", "constant");
          const spec = frequencySpecDefinitions[specKey] || frequencySpecDefinitions.tracking;
          const plant = frequencySpecPlants[plantKey] || frequencySpecPlants.constant;
          const freqExp = val(activity, "spec-frequency-exp", -1);
          const omega = 10 ** freqExp;
          const specLevelInput = activity.querySelector("[data-input='spec-level-db']");
          if (specLevelInput && specLevelInput.dataset.specKey !== specKey) {
            specLevelInput.value = String(spec.defaultLevelDb ?? 20);
            specLevelInput.dataset.specKey = specKey;
          }
          const specDb = val(activity, "spec-level-db", spec.defaultLevelDb ?? 20);
          const offsetDb = val(activity, "loop-offset-db", 20);
          const amplitude = val(activity, "spec-amplitude", 1);
          const plantDb = plant.magDb(omega);
          const loopDb = plantDb + offsetDb;
          const guideDb = spec.guide(omega, plantDb);
          const boundaryDb = (guideDb === null ? 0 : guideDb) + specDb;
          const metricDb = guideDb === null ? loopDb : loopDb - guideDb;
          const marginDb = spec.accepts === "below" ? boundaryDb - loopDb : loopDb - boundaryDb;
          const meetsSpec = marginDb >= 0;
          const metric = 10 ** (metricDb / 20);
          let bound = 10 ** (-metricDb / 20);
          let boundText = `<span class="metric-equation">${spec.boundLabel} = ${fmt(bound, 4)}</span>`;
          let note = spec.note(bound, metric);

          if (specKey === "noise") {
            bound = 10 ** (loopDb / 20);
            boundText = `<span class="metric-equation">${spec.boundLabel} = ${fmt(bound, 4)}</span>`;
            note = spec.note(bound, metric);
          }
          if (specKey === "type0") {
            bound = amplitude / (1 + metric);
            boundText = `<span class="metric-equation">e<sub>ss</sub> = A/(1 + M<sub>p</sub>) = ${fmt(bound, 4)}</span>`;
            note = spec.note(bound, metric);
          }
          if (specKey === "type1") {
            bound = amplitude / metric;
            boundText = `<span class="metric-equation">e<sub>ss</sub> = A/M<sub>v</sub> = ${fmt(bound, 4)}</span>`;
            note = spec.note(bound, metric);
          }
          if (specKey === "type2") {
            bound = amplitude / metric;
            boundText = `<span class="metric-equation">e<sub>ss</sub> = A/M<sub>a</sub> = ${fmt(bound, 4)}</span>`;
            note = spec.note(bound, metric);
          }

          setText(activity, "freq-exp", fmt(freqExp, 2));
          setText(activity, "spec-frequency", fmt(omega, 3));
          setHTML(activity, "spec-level-label", `<span class="inline-equation">${spec.measurement.split(" ")[0]}</span>`);
          setText(activity, "spec-level-db", fmt(specDb, 1));
          setText(activity, "loop-offset-db", fmt(offsetDb, 1));
          setText(activity, "spec-amplitude", fmt(amplitude, 2));
          setHTML(
            activity,
            "spec-measurement",
            `<span class="metric-equation">${spec.measurement}: ${fmt(metricDb, 2)} dB at <span class="inline-equation">${spec.frequencyLabel} = ${fmt(omega, 3)}</span> rad/sec. Target: <span class="inline-equation">${fmt(specDb, 1)} dB</span>; margin to boundary: <span class="inline-equation">${fmt(marginDb, 2)} dB</span> (${meetsSpec ? "acceptable" : "not acceptable"}). <span class="inline-equation">&vert;P&vert; = ${fmt(plantDb, 2)} dB</span>, <span class="inline-equation">&vert;PC&vert; = ${fmt(loopDb, 2)} dB</span> for ${plant.labelHtml}.</span>`
          );
          setHTML(activity, "spec-bound", boundText);
          setHTML(activity, "spec-note", note);
          drawFrequencySpecCanvas(activity, { plant, spec, omega, offsetDb, plantDb, loopDb, guideDb, boundaryDb, metricDb, specDb, meetsSpec });
        }
        if (type === "sensitivity") {
          const loopDb = val(activity, "loop-gain", 20);
          const l = 10 ** (loopDb / 20);
          const s = 1 / (1 + l);
          const t = l / (1 + l);
          setText(activity, "sdb", `${fmt(20 * Math.log10(s), 1)} dB`);
          setText(activity, "tdb", `${fmt(20 * Math.log10(t), 1)} dB`);
          setText(activity, "tracking", loopDb >= 20 ? "good low-frequency tracking/disturbance rejection" : "weak low-frequency tracking/disturbance rejection");
          setText(activity, "noise", loopDb <= -10 ? "good high-frequency noise attenuation" : "noise passes through unless gain rolls off");
          drawActivityVisual(activity, type, { loopDb, sDb: `${fmt(20 * Math.log10(s), 1)} dB`, tDb: `${fmt(20 * Math.log10(t), 1)} dB` });
        }
        if (type === "margins") {
          const gainShift = val(activity, "gain-shift", 0);
          const lag = val(activity, "phase-lag", 40);
          const xMin = -1.1;
          const xMax = 2.2;
          const magDb = (omega) => (
            gainShift
            + 12
            - 20 * Math.log10(omega)
            - 10 * Math.log10(1 + (omega / 6) ** 2)
            - 10 * Math.log10(1 + (omega / 40) ** 2)
          );
          const phaseDeg = (omega) => (
            -90
            - lag
            - Math.atan(omega / 6) * 180 / Math.PI
            - Math.atan(omega / 40) * 180 / Math.PI
          );
          const findCrossing = (fn, target) => {
            let previousLog = xMin;
            let previous = fn(10 ** previousLog) - target;
            for (let i = 1; i <= 260; i += 1) {
              const currentLog = xMin + ((xMax - xMin) * i) / 260;
              const current = fn(10 ** currentLog) - target;
              if (previous === 0) return 10 ** previousLog;
              if (previous * current <= 0) {
                let lo = previousLog;
                let hi = currentLog;
                let flo = previous;
                for (let j = 0; j < 36; j += 1) {
                  const mid = (lo + hi) / 2;
                  const fmid = fn(10 ** mid) - target;
                  if (flo * fmid <= 0) {
                    hi = mid;
                  } else {
                    lo = mid;
                    flo = fmid;
                  }
                }
                return 10 ** ((lo + hi) / 2);
              }
              previousLog = currentLog;
              previous = current;
            }
            return NaN;
          };
          const gainCrossover = findCrossing(magDb, 0);
          const phaseCrossover = findCrossing(phaseDeg, -180);
          const phaseAtGainCrossover = Number.isFinite(gainCrossover) ? phaseDeg(gainCrossover) : NaN;
          const magAtPhaseCrossover = Number.isFinite(phaseCrossover) ? magDb(phaseCrossover) : NaN;
          const pm = Number.isFinite(phaseAtGainCrossover) ? 180 + phaseAtGainCrossover : NaN;
          const gm = Number.isFinite(magAtPhaseCrossover) ? -magAtPhaseCrossover : NaN;
          setText(activity, "gain-shift", String(Math.round(gainShift)));
          setText(activity, "phase-lag", String(Math.round(lag)));
          setText(activity, "wcg", Number.isFinite(gainCrossover) ? `${fmt(gainCrossover, 2)} rad/s` : "none in plotted range");
          setText(activity, "wcp", Number.isFinite(phaseCrossover) ? `${fmt(phaseCrossover, 2)} rad/s` : "none in plotted range");
          setText(activity, "pm", Number.isFinite(pm) ? `${fmt(pm, 1)} deg` : "not defined");
          setText(activity, "gm", Number.isFinite(gm) ? `${fmt(gm, 1)} dB` : "infinite / not defined");
          setText(activity, "margin-note", Number.isFinite(pm) && pm > 35 && (!Number.isFinite(gm) || gm > 6) ? "comfortable positive margins" : "one margin is small: robustness is poor");
          drawActivityVisual(activity, type, {
            gainShift,
            phaseLag: lag,
            phaseAtGainCrossover,
            magAtPhaseCrossover,
            pmValue: pm,
            gmValue: gm,
            gainCrossoverValue: gainCrossover,
            phaseCrossoverValue: phaseCrossover,
            pm: Number.isFinite(pm) ? `${fmt(pm, 1)} deg` : "not defined",
            gm: Number.isFinite(gm) ? `${fmt(gm, 1)} dB` : "infinite",
            gainCrossover: Number.isFinite(gainCrossover) ? `${fmt(gainCrossover, 2)} rad/s` : "none",
            phaseCrossover: Number.isFinite(phaseCrossover) ? `${fmt(phaseCrossover, 2)} rad/s` : "none",
          });
        }
        if (type === "ch17-phase-response") {
          const pm = val(activity, "ch17-phase-margin", 60);
          const zeta = Math.max(0.18, Math.min(1.15, pm / 85));
          const peakDb = zeta < 1 / Math.sqrt(2) ? 20 * Math.log10(1 / (2 * zeta * Math.sqrt(1 - zeta * zeta))) : 0;
          const overshoot = zeta < 1 ? 100 * Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta)) : 0;
          const interpretation = pm < 45
            ? "Low margin: the response is underdamped, so peaking and ringing are likely."
            : pm <= 70
              ? "Typical design range: useful damping without giving up too much responsiveness."
              : "High margin: the response is well damped, but the design may be conservative.";
          setText(activity, "ch17-phase-margin", String(Math.round(pm)));
          setText(activity, "ch17-phase-zeta", `ζ ≈ ${fmt(zeta, 2)}`);
          setText(activity, "ch17-phase-peak", `${fmt(peakDb, 1)} dB`);
          setText(activity, "ch17-phase-overshoot", `${fmt(overshoot, 1)}%`);
          setText(activity, "ch17-phase-interpretation", interpretation);
          setText(activity, "ch17-phase-approximation", "Second-order trend model: ζ ≈ PM / 85.");
          drawActivityVisual(activity, type, { pm });
        }
        if (type === "ch17-crossover-effort") {
          const exp = val(activity, "ch17-crossover-exp", 1);
          const wc = 10 ** exp;
          const settling = 4 / (0.68 * wc);
          const peakU = 0.65 * wc;
          setText(activity, "ch17-crossover-value", fmt(wc, 2));
          setText(activity, "ch17-crossover-settling", `t_s ≈ 4/(0.68 ω_co) = ${fmt(settling, 2)} s`);
          setText(activity, "ch17-crossover-peak-u", `peak proxy ≈ 0.65 ω_co = ${fmt(peakU, 1)}`);
          setText(activity, "ch17-crossover-note", peakU > 35 ? "High actuator demand: the response is fast, but the example effort limit is exceeded." : wc < 5 ? "Low actuator demand, but the response is slow." : "Moderate actuator demand with a useful speed/effort tradeoff.");
          drawActivityVisual(activity, type, { wc });
        }
        if (type === "ch17-loopshape-checklist") {
          const lowGain = val(activity, "ch17-loopshape-low-gain", 50);
          const wcExp = val(activity, "ch17-loopshape-wc-exp", 0.15);
          const wc = 10 ** wcExp;
          const slopeMag = val(activity, "ch17-loopshape-slope", 40);
          const rolloff = val(activity, "ch17-loopshape-rolloff", 25);
          const pm = Math.max(15, Math.min(95, 120 - 1.5 * slopeMag));
          const highNoiseDb = -slopeMag - (slopeMag + rolloff) * Math.max(0, Math.log10(100 / wc) - 1);
          const lowOk = lowGain >= 35;
          const slopeOk = slopeMag >= 20 && slopeMag <= 40;
          const effortOk = wc <= 2.5;
          const noiseOk = highNoiseDb <= -40;
          const passCount = [lowOk, slopeOk, effortOk, noiseOk].filter(Boolean).length;
          const active = activity.dataset.ch17LoopshapeActive || "low";
          const intFmt = (value) => String(Math.round(value));
          const statusFor = {
            low: lowOk ? `PASS: low-frequency loop gain is ${intFmt(lowGain)} dB.` : `ADJUST: raise low-frequency gain above about 35 dB.`,
            slope: slopeOk ? `PASS: crossover slope is -${intFmt(slopeMag)} dB/dec, in the -20 to -40 dB/dec target range.` : `ADJUST: use a slope between about -20 and -40 dB/dec near crossover.`,
            effort: effortOk ? `PASS: ω_co = ${fmt(wc, 2)} rad/s stays below the example saturation limit.` : `ADJUST: ω_co is high, so actuator saturation is more likely.`,
            noise: noiseOk ? `PASS: high-frequency loop gain is about ${fmt(highNoiseDb, 1)} dB at 100 rad/s.` : `ADJUST: add high-frequency rolloff so |PC| is well below 0 dB where noise dominates.`,
          };
          setText(activity, "ch17-loopshape-low-gain", intFmt(lowGain));
          setText(activity, "ch17-loopshape-wc", fmt(wc, 2));
          setText(activity, "ch17-loopshape-slope", `-${intFmt(slopeMag)}`);
          setText(activity, "ch17-loopshape-rolloff", intFmt(rolloff));
          setText(activity, "ch17-loopshape-status", `${passCount} / 4 checks pass. ${statusFor[active] || statusFor.low} Estimated phase margin is ${intFmt(pm)} degrees.`);
          drawActivityVisual(activity, type, {
            lowGain,
            wc,
            slopeMag,
            rolloff,
            active,
          });
        }
        if (type === "compensator-score") {
          const gainDb = val(activity, "dashboard-gain-db", 6);
          const piZero = 10 ** val(activity, "dashboard-pi-zero-exp", -0.4);
          const lagZero = 10 ** val(activity, "dashboard-lag-zero-exp", -0.1);
          const lagRatio = val(activity, "dashboard-lag-ratio", 10);
          const leadCenter = 10 ** val(activity, "dashboard-lead-center-exp", 0.55);
          const leadRatio = val(activity, "dashboard-lead-ratio", 12);
          const lpfPole = 10 ** val(activity, "dashboard-lpf-pole-exp", 0.9);
          const params = { gainDb, piZero, lagZero, lagRatio, leadCenter, leadRatio, lpfPole };
          const margin = ch18FindDashboardCrossover(params);
          const lowOmega = 0.03;
          const noiseOmega = 100;
          const lowGain = ch18DashboardResponse(params, lowOmega).magDb;
          const noiseGain = ch18DashboardResponse(params, noiseOmega).magDb;
          const lowOk = lowGain >= 30;
          const marginOk = Number.isFinite(margin.phaseMargin) && margin.phaseMargin >= 45;
          const bandwidthOk = Number.isFinite(margin.wc) && margin.wc >= 0.5 && margin.wc <= 20;
          const noiseOk = noiseGain <= -20;
          const checksPassed = [lowOk, marginOk, bandwidthOk, noiseOk].filter(Boolean).length;
          const badge = (ok) => ok ? "PASS" : "ADJUST";
          let nextMove = "All four checks pass. Compare whether a simpler controller could meet the same requirements.";
          if (!lowOk) nextMove = "Increase K, raise low-frequency lag boost, or move the PI zero up until the low-frequency loop gain clears the green guide.";
          else if (!marginOk) nextMove = "Add lead near crossover, reduce K, or move lag/low-pass corners farther from crossover to recover phase margin.";
          else if (!bandwidthOk && Number.isFinite(margin.wc) && margin.wc < 0.5) nextMove = "Increase K or move the lead center upward to raise bandwidth.";
          else if (!bandwidthOk) nextMove = "Reduce K or add more high-frequency rolloff so crossover does not move too high.";
          else if (!noiseOk) nextMove = "Lower the low-pass pole or reduce high-frequency lead gain until |PC| is below the noise guide.";

          setText(activity, "dashboard-gain-k", fmt(10 ** (gainDb / 20), 3));
          setText(activity, "dashboard-gain-db", fmt(gainDb, 0));
          setText(activity, "dashboard-pi-zero", fmt(piZero, 3));
          setText(activity, "dashboard-lag-zero", fmt(lagZero, 3));
          setText(activity, "dashboard-lag-ratio", fmt(lagRatio, 0));
          setText(activity, "dashboard-lead-center", fmt(leadCenter, 3));
          setText(activity, "dashboard-lead-ratio", fmt(leadRatio, 0));
          setText(activity, "dashboard-lpf-pole", fmt(lpfPole, 3));
          setText(activity, "dashboard-low-gain", `${badge(lowOk)}: |PC(j${fmt(lowOmega, 2)})| = ${fmt(lowGain, 1)} dB; target >= 30 dB`);
          setText(activity, "dashboard-phase-margin", Number.isFinite(margin.phaseMargin) ? `${badge(marginOk)}: PM = ${fmt(margin.phaseMargin, 1)} deg at wc = ${fmt(margin.wc, 2)} rad/s` : "ADJUST: no gain crossover in plotted range");
          setText(activity, "dashboard-bandwidth", Number.isFinite(margin.wc) ? `${badge(bandwidthOk)}: wc = ${fmt(margin.wc, 2)} rad/s; target 0.5 to 20 rad/s` : "ADJUST: no crossover to measure bandwidth");
          setText(activity, "dashboard-noise", `${badge(noiseOk)}: |PC(j${fmt(noiseOmega, 0)})| = ${fmt(noiseGain, 1)} dB; target <= -20 dB`);
          setText(activity, "dashboard-controller", `${checksPassed} / 4 checks pass; K=${fmt(10 ** (gainDb / 20), 2)}, zPI=${fmt(piZero, 2)}, zlag=${fmt(lagZero, 2)}, Mlag=${fmt(lagRatio, 0)}, wlead=${fmt(leadCenter, 2)}, Mlead=${fmt(leadRatio, 0)}, pLPF=${fmt(lpfPole, 2)}`);
          setText(activity, "dashboard-next-move", nextMove);
          drawActivityVisual(activity, type, {
            params,
            metrics: { margin, lowOk, marginOk, bandwidthOk, noiseOk },
          });
        }
      };
      const safeUpdate = () => {
        try {
          update();
        } catch (error) {
          console.error(`Advanced activity failed: ${type}`, error);
        }
      };
      for (const input of activity.querySelectorAll("[data-input]")) {
        input.addEventListener("input", safeUpdate);
        input.addEventListener("change", safeUpdate);
      }
      for (const input of activity.querySelectorAll("[data-select]")) {
        input.addEventListener("change", safeUpdate);
      }
      if (type === "system-type") {
        const checkButton = activity.querySelector("[data-action='system-type-check']");
        if (checkButton) {
          checkButton.addEventListener("click", () => {
            activity.dataset.systemTypeChecked = "true";
            safeUpdate();
          });
        }
        const resetClassification = () => {
          activity.dataset.systemTypeChecked = "false";
          safeUpdate();
        };
        for (const control of activity.querySelectorAll("[data-select='system-type-example'], [data-input='system-type-w1'], [data-input='system-type-w2']")) {
          control.addEventListener("input", resetClassification);
          control.addEventListener("change", resetClassification);
        }
      }
      if (type === "ch15-example-1-builder") {
        const checkButton = activity.querySelector("[data-action='ex1-check-predictions']");
        if (checkButton) {
          checkButton.addEventListener("click", () => {
            activity.dataset.ex1PredictionChecked = "true";
            safeUpdate();
          });
        }
        const resetPrediction = () => {
          activity.dataset.ex1PredictionChecked = "false";
          safeUpdate();
        };
        for (const control of activity.querySelectorAll("[data-select='ex1-slope-predict'], [data-select='ex1-phase-predict']")) {
          control.addEventListener("change", resetPrediction);
        }
      }
      if (type === "ch15-bode-example-builder") {
        const checkButton = activity.querySelector("[data-action='example-check-predictions']");
        if (checkButton) {
          checkButton.addEventListener("click", () => {
            activity.dataset.examplePredictionChecked = "true";
            safeUpdate();
          });
        }
        const resetPrediction = () => {
          activity.dataset.examplePredictionChecked = "false";
          safeUpdate();
        };
        for (const control of activity.querySelectorAll("[data-select='example'], [data-select='example-slope-predict'], [data-select='example-phase-predict']")) {
          control.addEventListener("change", resetPrediction);
        }
      }
      if (type === "ch17-loopshape-checklist") {
        const loopshapeDetails = {
          low: {
            title: "Low-frequency gain",
            goal: "Design goal: make |PC| much larger than 1 at low frequency.",
            why: "Large low-frequency loop gain makes the sensitivity function small where references, input disturbances, and output disturbances usually have most of their energy.",
            fix: "Check that |PC| is well above 0 dB on the left side of the Bode plot. Integral action, lag compensation, or higher DC gain can help, but they can also move crossover or reduce phase margin.",
            question: "Which design action best supports low-frequency tracking and disturbance rejection?",
            answers: {
              a: "Raise low-frequency loop gain",
              b: "Raise high-frequency sensor gain",
              c: "Make crossover slope steeper",
            },
            correct: "a",
            feedback: "Large low-frequency loop gain makes S small, which improves tracking and disturbance rejection.",
          },
          slope: {
            title: "Crossover slope",
            goal: "Design goal: keep the slope near crossover shallow enough for useful phase margin.",
            why: "The Bode phase-gain relationship ties local magnitude slope to phase. A slope that is too steep near the 0 dB crossing usually means too much phase lag and poor robustness.",
            fix: "Check the slope for roughly a decade around crossover. A common target is about -20 to -40 dB/dec; avoid a long -60 dB/dec region through crossover.",
            question: "What usually happens if the magnitude slope is about -60 dB/dec at crossover?",
            answers: {
              a: "Phase margin tends to improve",
              b: "Phase margin tends to be poor",
              c: "Noise attenuation disappears at DC",
            },
            correct: "b",
            feedback: "A steep negative slope near crossover usually corresponds to large phase lag, so phase margin gets smaller.",
          },
          effort: {
            title: "Crossover frequency and actuator effort",
            goal: "Design goal: choose crossover high enough for speed, but not so high that the actuator saturates.",
            why: "Increasing crossover usually speeds up the closed-loop response, but it also increases control effort and can amplify unmodeled high-frequency dynamics.",
            fix: "Check actuator limits and simulated control input. If the response is fast but the actuator saturates, lower crossover or reshape the controller.",
            question: "What practical warning says crossover may be too high?",
            answers: {
              a: "Control input approaches saturation",
              b: "Low-frequency gain is above 0 dB",
              c: "The plant has a DC gain",
            },
            correct: "a",
            feedback: "If the actuator saturates, the linear loop-shape prediction no longer describes the real closed-loop behavior.",
          },
          noise: {
            title: "High-frequency attenuation",
            goal: "Design goal: make |PC| small at high frequency.",
            why: "Sensor noise often lives at high frequency. If loop gain remains large there, the controller can inject noisy control action instead of attenuating it.",
            fix: "Check that the loop rolls off after the useful bandwidth. Low-pass filtering and avoiding unnecessary high-frequency gain help reduce noise transmission.",
            question: "Why should loop gain be small where sensor noise is dominant?",
            answers: {
              a: "To increase DC tracking error",
              b: "To prevent noise-driven control action",
              c: "To force the loop to cross 0 dB twice",
            },
            correct: "b",
            feedback: "Low high-frequency loop gain helps keep sensor noise from driving the actuator.",
          },
        };
        const showLoopshapeDetail = (key) => {
          const detail = loopshapeDetails[key] || loopshapeDetails.low;
          activity.dataset.ch17LoopshapeActive = key;
          setText(activity, "ch17-loopshape-title", detail.title);
          setText(activity, "ch17-loopshape-goal", detail.goal);
          setText(activity, "ch17-loopshape-why", detail.why);
          setText(activity, "ch17-loopshape-fix", detail.fix);
          setText(activity, "ch17-loopshape-question", detail.question);
          const feedback = activity.querySelector('[data-output="ch17-loopshape-feedback"]');
          feedback?.classList.remove("is-correct", "is-wrong");
          setText(activity, "ch17-loopshape-feedback", "");
          for (const hotspot of activity.querySelectorAll("[data-ch17-loopshape-hotspot]")) {
            hotspot.classList.toggle("is-active", hotspot.dataset.ch17LoopshapeHotspot === key);
          }
          for (const answer of activity.querySelectorAll("[data-ch17-loopshape-answer]")) {
            const option = answer.dataset.ch17LoopshapeAnswer;
            answer.textContent = detail.answers[option] || "";
            answer.classList.remove("is-correct", "is-wrong");
          }
        };
        for (const hotspot of activity.querySelectorAll("[data-ch17-loopshape-hotspot]")) {
          hotspot.addEventListener("click", () => {
            showLoopshapeDetail(hotspot.dataset.ch17LoopshapeHotspot || "low");
            safeUpdate();
          });
        }
        for (const answer of activity.querySelectorAll("[data-ch17-loopshape-answer]")) {
          answer.addEventListener("click", () => {
            const active = activity.dataset.ch17LoopshapeActive || "low";
            const detail = loopshapeDetails[active] || loopshapeDetails.low;
            const ok = answer.dataset.ch17LoopshapeAnswer === detail.correct;
            for (const option of activity.querySelectorAll("[data-ch17-loopshape-answer]")) {
              const optionOk = option.dataset.ch17LoopshapeAnswer === detail.correct;
              option.classList.toggle("is-correct", optionOk);
              option.classList.toggle("is-wrong", option === answer && !ok);
            }
            const feedback = activity.querySelector('[data-output="ch17-loopshape-feedback"]');
            feedback?.classList.toggle("is-correct", ok);
            feedback?.classList.toggle("is-wrong", !ok);
            setText(activity, "ch17-loopshape-feedback", ok ? `Correct. ${detail.feedback}` : `Not quite. ${detail.feedback}`);
          });
        }
        showLoopshapeDetail("low");
      }
      if (type === "ch18-loopshape-sequence") {
        activity.dataset.ch18SequenceStep = activity.dataset.ch18SequenceStep || "0";
        for (const button of activity.querySelectorAll("[data-ch18-sequence-step]")) {
          button.addEventListener("click", () => {
            activity.dataset.ch18SequenceStep = button.dataset.ch18SequenceStep || "0";
            safeUpdate();
          });
        }
      }
      if (type === "ch18-slc") {
        const stepSelect = activity.querySelector("[data-select='ch18-slc-step']");
        for (const button of activity.querySelectorAll("[data-ch18-slc-step]")) {
          button.addEventListener("click", () => {
            if (stepSelect) {
              stepSelect.value = button.dataset.ch18SlcStep || "inner";
            }
            safeUpdate();
          });
        }
      }
      safeUpdate();
      if (activity.querySelector("canvas[data-plot]")) {
        const redraw = () => window.requestAnimationFrame(() => safeUpdate());
        window.addEventListener("load", redraw, { once: true });
        window.addEventListener("resize", redraw);
        window.setTimeout(redraw, 0);
        window.setTimeout(redraw, 250);
        if ("IntersectionObserver" in window) {
          const observer = new IntersectionObserver((entries) => {
            if (entries.some((entry) => entry.isIntersecting)) redraw();
          }, { threshold: 0.01 });
          observer.observe(activity);
        }
      }
        } catch (error) {
          console.error(`Advanced activity setup failed: ${activity?.dataset?.advancedActivity || "unknown"}`, error);
        }
    }
  };

  const initGuidedDesignActivities = () => {
    const expectedByActivity = {
      "ch11-state-feedback": {
        controllability: { c11: 0, c12: 2, c21: 2, c22: 4, detc: -4 },
        "open-loop": { a1: -2, a0: -1 },
        desired: { alpha1: 4, alpha0: 5 },
        gain: { k1: 3, k2: 3 },
        "closed-loop": { acl11: 0, acl12: 1, acl21: -5, acl22: -4 },
        "reference-gain": { kr: 5 / 6 },
      },
      "ch12-integrator-feedback": {
        augmented: {
          a11: 0, a12: 1, a13: 0,
          a21: -2, a22: -3, a23: 0,
          a31: -1, a32: 0, a33: 0,
          b1: 0, b2: 1, b3: 0,
        },
        controllability: {
          c11: 0, c12: 1, c13: -3,
          c21: 1, c22: -3, c23: 7,
          c31: 0, c32: 0, c33: -1,
          detc: 1,
        },
        "open-loop": { a2: 3, a1: 2, a0: 0 },
        desired: { alpha2: 5, alpha1: 9, alpha0: 5 },
        gain: { k1: 7, k2: 2, ki: -5 },
        extract: { kx1: 7, kx2: 2, kiextract: -5 },
      },
      "ch13-observer": {
        observability: { o11: 2, o12: 0, o21: 0, o22: 2, rank: 2 },
        "open-loop": { a1: 5, a0: 6 },
        desired: { beta1: 8, beta0: 25 },
        gain: { l1: 1.5, l2: 2 },
        poles: { trace: -8, det: 25 },
      },
    };
    const aliases = {
      "3/2": 1.5,
      "6/4": 1.5,
      "4/2": 2,
      "25/1": 25,
    };
    const parseNumeric = (value) => {
      const text = String(value || "").trim().toLowerCase();
      if (Object.prototype.hasOwnProperty.call(aliases, text)) return aliases[text];
      const fraction = text.match(/^(-?\d+(?:\.\d+)?)\s*\/\s*(-?\d+(?:\.\d+)?)$/);
      if (fraction) {
        const numerator = Number(fraction[1]);
        const denominator = Number(fraction[2]);
        return denominator === 0 ? NaN : numerator / denominator;
      }
      return Number(text);
    };

    for (const activity of document.querySelectorAll("[data-guided-design], [data-guided-observer]")) {
      const activityKey = activity.dataset.guidedDesign || "ch13-observer";
      const expected = expectedByActivity[activityKey] || expectedByActivity["ch13-observer"];
      for (const button of activity.querySelectorAll("[data-guided-check]")) {
        const step = button.dataset.guidedCheck;
        const targets = expected[step] || {};
        const feedback = activity.querySelector(`[data-guided-feedback="${step}"]`);
        const answer = activity.querySelector(`[data-guided-answer="${step}"]`);
        if (answer) answer.hidden = true;
        button.addEventListener("click", () => {
          const missing = [];
          const wrong = [];
          for (const [key, target] of Object.entries(targets)) {
            const input = activity.querySelector(`[data-guided-input="${key}"]`);
            const value = parseNumeric(input?.value);
            input?.classList.remove("is-correct", "is-wrong");
            if (!input || input.value.trim() === "" || Number.isNaN(value)) {
              missing.push(key);
              input?.classList.add("is-wrong");
            } else if (Math.abs(value - target) <= 0.001) {
              input.classList.add("is-correct");
            } else {
              wrong.push(key);
              input.classList.add("is-wrong");
            }
          }
          const ok = missing.length === 0 && wrong.length === 0;
          if (feedback) {
            feedback.textContent = ok
              ? "Correct. The solution details are shown below."
              : "Some entries need correction. Compare your work with the solution details below, then revise before continuing.";
            feedback.classList.toggle("is-correct", ok);
            feedback.classList.toggle("is-wrong", !ok);
          }
          if (answer) answer.hidden = false;
        });
        for (const key of Object.keys(targets)) {
          const input = activity.querySelector(`[data-guided-input="${key}"]`);
          if (!input) continue;
          input.addEventListener("input", () => {
            input.classList.remove("is-correct", "is-wrong");
            if (feedback) {
              feedback.textContent = "";
              feedback.classList.remove("is-correct", "is-wrong");
            }
            if (answer) answer.hidden = true;
          });
        }
      }
    }
  };

  const initDesignStudyAActivities = () => {
    const typesetMath = (root) => {
      if (!root || !window.MathJax?.typesetPromise) return;
      window.MathJax.typesetPromise([root]).catch(() => {});
    };

    const normalizeSymbolic = (value) => String(value || "")
      .trim()
      .toLowerCase()
      .replace(/[\u2018\u2019]/g, "'")
      .replace(/^(?:p|p_cm|pcm|r|r_cm|rcm|position|com|center\s+of\s+mass)\s*=\s*/i, "")
      .replace(/\|f\|\s*(?:<=|≤|\\leq|\\le)\s*5/g, "forcelimit5")
      .replace(/abs\s*\(\s*f\s*\)\s*(?:<=|≤|\\leq|\\le)\s*5/g, "forcelimit5")
      .replace(/cartposition/g, "z")
      .replace(/cart position/g, "z")
      .replace(/cartlocation/g, "z")
      .replace(/cart location/g, "z")
      .replace(/positionofthecart/g, "z")
      .replace(/position of the cart/g, "z")
      .replace(/pendulumangle/g, "theta")
      .replace(/pendulum angle/g, "theta")
      .replace(/angleofthependulum/g, "theta")
      .replace(/angle of the pendulum/g, "theta")
      .replace(/anglefromvertical/g, "theta")
      .replace(/angle from vertical/g, "theta")
      .replace(/bodyattitude/g, "theta")
      .replace(/body attitude/g, "theta")
      .replace(/bodyangle/g, "theta")
      .replace(/body angle/g, "theta")
      .replace(/satelliteattitude/g, "theta")
      .replace(/satellite attitude/g, "theta")
      .replace(/satellitebodyangle/g, "theta")
      .replace(/satellite body angle/g, "theta")
      .replace(/panelattitude/g, "phi")
      .replace(/panel attitude/g, "phi")
      .replace(/panelangle/g, "phi")
      .replace(/panel angle/g, "phi")
      .replace(/solar panel angle/g, "phi")
      .replace(/solarpanelangle/g, "phi")
      .replace(/flexibledeflection/g, "phiminustheta")
      .replace(/flexible deflection/g, "phiminustheta")
      .replace(/relativedeflection/g, "phiminustheta")
      .replace(/relative deflection/g, "phiminustheta")
      .replace(/paneldeflection/g, "phiminustheta")
      .replace(/panel deflection/g, "phiminustheta")
      .replace(/bodytorque/g, "tau")
      .replace(/body torque/g, "tau")
      .replace(/satellitetorque/g, "tau")
      .replace(/satellite torque/g, "tau")
      .replace(/torqueinput/g, "tau")
      .replace(/torque input/g, "tau")
      .replace(/thetadot/g, "thetadot")
      .replace(/theta dot/g, "thetadot")
      .replace(/theta_dot/g, "thetadot")
      .replace(/dottheta/g, "thetadot")
      .replace(/dot theta/g, "thetadot")
      .replace(/dtheta\/dt/g, "thetadot")
      .replace(/thetaderivative/g, "thetadot")
      .replace(/theta derivative/g, "thetadot")
      .replace(/thetavelocity/g, "thetadot")
      .replace(/theta velocity/g, "thetadot")
      .replace(/angularvelocity/g, "thetadot")
      .replace(/angular velocity/g, "thetadot")
      .replace(/zdot/g, "zdot")
      .replace(/z dot/g, "zdot")
      .replace(/z_dot/g, "zdot")
      .replace(/dotz/g, "zdot")
      .replace(/dot z/g, "zdot")
      .replace(/dz\/dt/g, "zdot")
      .replace(/zderivative/g, "zdot")
      .replace(/z derivative/g, "zdot")
      .replace(/cartvelocity/g, "zdot")
      .replace(/cart velocity/g, "zdot")
      .replace(/horizontalvelocity/g, "zdot")
      .replace(/horizontal velocity/g, "zdot")
      .replace(/\s+/g, "")
      .replace(/−/g, "-")
      .replace(/β/g, "beta")
      .replace(/&beta;/g, "beta")
      .replace(/\\beta/g, "beta")
      .replace(/α/g, "alpha")
      .replace(/\\alpha/g, "alpha")
      .replace(/θ/g, "theta")
      .replace(/\\theta/g, "theta")
      .replace(/φ/g, "phi")
      .replace(/\\phi/g, "phi")
      .replace(/τ/g, "tau")
      .replace(/\\tau/g, "tau")
      .replace(/ℓ/g, "l")
      .replace(/\\ell/g, "l")
      .replace(/\\([*^|()[\]/])/g, "$1")
      .replace(/\\/g, "")
      .replace(/[()[\]{}]/g, "")
      .replace(/\^t/g, "")
      .replace(/,/g, "")
      .replace(/;/g, "")
      .replace(/×/g, "*");

    const expectedByActivity = {
      "hw6-state-space": {
        "state-derivatives": {
          "x1dot-x2": ["1", "+1"],
          "x2dot-x2": ["-a", "-3b/(ml^2)", "-3*b/(m*l^2)", "-3b/(mell^2)", "-3*b/(m*ell^2)"],
          "x2dot-u": ["beta", "3/(ml^2)", "3/(m*l^2)", "3/(mell^2)", "3/(m*ell^2)"],
        },
        matrices: {
          a11: ["0"],
          a12: ["1", "+1"],
          a21: ["0"],
          a22: ["-a", "-3b/(ml^2)", "-3*b/(m*l^2)", "-3b/(mell^2)", "-3*b/(m*ell^2)"],
          b1: ["0"],
          b2: ["beta", "3/(ml^2)", "3/(m*l^2)", "3/(mell^2)", "3/(m*ell^2)"],
          c1: ["1", "+1"],
          c2: ["0"],
          d: ["0"],
        },
        "transfer-function": {
          "tf-num": ["beta", "3/(ml^2)", "3/(m*l^2)", "3/(mell^2)", "3/(m*ell^2)"],
          "tf-s": ["a", "3b/(ml^2)", "3*b/(m*l^2)", "3b/(mell^2)", "3*b/(m*ell^2)"],
          "tf-constant": ["0"],
        },
      },
    };

    const getStepTargets = (activity, step, fallbackTargets) => {
      if (Object.keys(fallbackTargets).length) return fallbackTargets;
      const stepEl = activity.querySelector(`[data-dsa-step="${step}"]`);
      const targets = {};
      for (const input of stepEl?.querySelectorAll("[data-dsa-input][data-dsa-accept]") || []) {
        targets[input.dataset.dsaInput || ""] = String(input.dataset.dsaAccept || "")
          .split("|")
          .map((item) => item.trim())
          .filter(Boolean);
      }
      return targets;
    };

    const extractIntroTokens = (value) => {
      const text = String(value || "")
        .toLowerCase()
        .replace(/[\u2018\u2019]/g, "'")
        .replace(/\\dot\{z\}/g, " zdot ")
        .replace(/\\dot\{theta\}/g, " thetadot ")
        .replace(/\\dot\{\\theta\}/g, " thetadot ")
        .replace(/θ/g, " theta ")
        .replace(/\\theta/g, " theta ")
        .replace(/dtheta\s*\/\s*dt/g, " thetadot ")
        .replace(/dz\s*\/\s*dt/g, " zdot ")
        .replace(/\btheta\s*dot\b/g, " thetadot ")
        .replace(/\bdot\s*theta\b/g, " thetadot ")
        .replace(/\bz\s*dot\b/g, " zdot ")
        .replace(/\bdot\s*z\b/g, " zdot ")
        .replace(/\bcart\s+(position|location)\b/g, " z ")
        .replace(/\bposition\s+of\s+the\s+cart\b/g, " z ")
        .replace(/\bpendulum\s+angle\b/g, " theta ")
        .replace(/\bangle\s+of\s+the\s+pendulum\b/g, " theta ")
        .replace(/\bangle\s+from\s+vertical\b/g, " theta ")
        .replace(/\bcart\s+velocity\b/g, " zdot ")
        .replace(/\bhorizontal\s+velocity\b/g, " zdot ")
        .replace(/\bangular\s+velocity\b/g, " thetadot ")
        .replace(/\btheta\s+velocity\b/g, " thetadot ");
      const tokens = new Set();
      for (const match of text.matchAll(/\b(theta_dot|thetadot|dottheta|theta|z_dot|zdot|dotz|z)\b/g)) {
        const token = match[1].replace("theta_dot", "thetadot").replace("dottheta", "thetadot").replace("z_dot", "zdot").replace("dotz", "zdot");
        tokens.add(token);
      }
      return tokens;
    };

    const hasAllTokens = (tokens, expected) => expected.every((token) => tokens.has(token));

    const passesFlexibleIntroCheck = (activity, key, rawValue) => {
      const tokens = extractIntroTokens(rawValue);
      if (activity.dataset.designStudyA === "intro-orientation") {
        if (key === "state") {
          return hasAllTokens(tokens, ["theta", "thetadot"]);
        }
        if (key === "output") {
          return tokens.has("theta") && !tokens.has("thetadot");
        }
        return false;
      }
      if (activity.dataset.designStudyB !== "intro-orientation") return false;
      if (key === "state") {
        return hasAllTokens(tokens, ["z", "theta", "zdot", "thetadot"]);
      }
      if (key === "outputs") {
        return hasAllTokens(tokens, ["z", "theta"]) && !tokens.has("zdot") && !tokens.has("thetadot");
      }
      return false;
    };

    const getIntroOrientationStepSet = (activity, step) => {
      if (activity.dataset.designStudyA === "intro-orientation" && step === "configuration") {
        return { keys: ["angle"], expected: ["theta"] };
      }
      if (activity.dataset.designStudyB === "intro-orientation" && step === "configuration") {
        return { keys: ["cart-position", "pendulum-angle"], expected: ["z", "theta"] };
      }
      if (activity.dataset.designStudyC === "intro-orientation" && step === "coordinates") {
        return { keys: ["body", "panel", "deflection"], expected: ["theta", "phi", "phiminustheta"] };
      }
      return null;
    };

    const tokenForIntroOrientationInput = (activity, key, rawValue) => {
      const value = normalizeSymbolic(rawValue);
      if (activity.dataset.designStudyA === "intro-orientation" && key === "angle") {
        return value === "theta" ? "theta" : "";
      }
      if (activity.dataset.designStudyB === "intro-orientation") {
        if (value === "z") return "z";
        if (value === "theta") return "theta";
        return "";
      }
      if (activity.dataset.designStudyC === "intro-orientation") {
        if (value === "theta") return "theta";
        if (value === "phi") return "phi";
        if (value === "phiminustheta" || value === "phitheta" || value === "phi-theta") return "phiminustheta";
        return "";
      }
      return "";
    };

    const evaluateIntroOrientationStepSet = (activity, step) => {
      const config = getIntroOrientationStepSet(activity, step);
      if (!config) return null;
      const values = config.keys.map((key) => {
        const input = activity.querySelector(`[data-dsa-input="${key}"]`);
        const rawValue = input?.value || "";
        return { key, input, token: tokenForIntroOrientationInput(activity, key, rawValue) };
      });
      const missing = values.filter((item) => !item.input || !item.token);
      const tokenCounts = new Map();
      for (const item of values) {
        if (!item.token) continue;
        tokenCounts.set(item.token, (tokenCounts.get(item.token) || 0) + 1);
      }
      const hasExpected = config.expected.every((token) => tokenCounts.get(token) === 1);
      const noExtra = values.every((item) => !item.token || config.expected.includes(item.token));
      const noDuplicates = Array.from(tokenCounts.values()).every((count) => count === 1);
      const ok = missing.length === 0 && hasExpected && noExtra && noDuplicates;
      const wrong = values.filter((item) => item.input && item.token && (!config.expected.includes(item.token) || tokenCounts.get(item.token) > 1));
      for (const item of values) {
        item.input?.classList.remove("is-correct", "is-wrong");
        if (ok) {
          item.input?.classList.add("is-correct");
        } else if (!item.input || !item.token || wrong.includes(item)) {
          item.input?.classList.add("is-wrong");
        }
      }
      return {
        ok,
        missing: missing.map((item) => item.key),
        wrong: wrong.map((item) => item.key),
      };
    };

    for (const activity of document.querySelectorAll(".guided-design-study[data-design-study-a], .guided-design-study[data-design-study-b], .guided-design-study[data-design-study-c]")) {
      const activityKey = activity.dataset.designStudyA || activity.dataset.designStudyB || activity.dataset.designStudyC || "";
      const expected = expectedByActivity[activityKey] || {};
      const completeSolution = activity.querySelector("[data-dsa-complete-solution]");
      const completeFeedback = activity.querySelector("[data-dsa-complete-feedback]");
      const revealComplete = (message = "Complete solution revealed.") => {
        if (completeSolution) {
          completeSolution.hidden = false;
          typesetMath(completeSolution);
        }
        if (completeFeedback) {
          completeFeedback.textContent = message;
          completeFeedback.classList.remove("is-wrong");
          completeFeedback.classList.add("is-correct");
        }
      };
      for (const answer of activity.querySelectorAll("[data-dsa-answer]")) answer.hidden = true;
      for (const button of activity.querySelectorAll("[data-dsa-check]")) {
        const step = button.dataset.dsaCheck || "";
        const targets = getStepTargets(activity, step, expected[step] || {});
        const feedback = activity.querySelector(`[data-dsa-feedback="${step}"]`);
        const answer = activity.querySelector(`[data-dsa-answer="${step}"]`);
        button.addEventListener("click", () => {
          const missing = [];
          const wrong = [];
          const introStepSetResult = evaluateIntroOrientationStepSet(activity, step);
          if (introStepSetResult) {
            missing.push(...introStepSetResult.missing);
            wrong.push(...introStepSetResult.wrong);
          } else {
            for (const [key, acceptedRaw] of Object.entries(targets)) {
              const input = activity.querySelector(`[data-dsa-input="${key}"]`);
              const rawValue = input?.value || "";
              const value = normalizeSymbolic(rawValue);
              const accepted = acceptedRaw.map(normalizeSymbolic);
              input?.classList.remove("is-correct", "is-wrong");
              if (!input || !value) {
                missing.push(key);
                input?.classList.add("is-wrong");
              } else if (accepted.includes(value) || passesFlexibleIntroCheck(activity, key, rawValue)) {
                input.classList.add("is-correct");
              } else {
                wrong.push(key);
                input.classList.add("is-wrong");
              }
            }
          }
          const ok = introStepSetResult
            ? introStepSetResult.ok
            : missing.length === 0 && wrong.length === 0;
          if (feedback) {
            const hint = button.dataset.dsaHint || "";
            feedback.textContent = ok
              ? "Correct. The solution detail for this step is shown below."
              : activity.hasAttribute("data-dsa-show-solution-on-wrong")
                ? `Some entries need correction. ${hint} Select "Show solution" below if you want to see the answer.`
                : `Some entries need correction.${hint ? ` ${hint}` : " Revise the highlighted entries, then check again."}`;
            feedback.classList.toggle("is-correct", ok);
            feedback.classList.toggle("is-wrong", !ok);
          }
          if (answer) {
            answer.hidden = false;
            if (!ok && activity.hasAttribute("data-dsa-open-answer-on-wrong")) {
              answer.open = true;
            } else if (!ok && activity.hasAttribute("data-dsa-show-solution-on-wrong")) {
              answer.open = false;
            }
            typesetMath(answer);
          }
          const allChecks = Array.from(activity.querySelectorAll("[data-dsa-check]"));
          const allPassed = allChecks.every((checkButton) => {
            const checkedStep = checkButton.dataset.dsaCheck || "";
            const introStepSetResult = evaluateIntroOrientationStepSet(activity, checkedStep);
            if (introStepSetResult) return introStepSetResult.ok;
            const checkedTargets = getStepTargets(activity, checkedStep, expected[checkedStep] || {});
            return Object.entries(checkedTargets).every(([key, acceptedRaw]) => {
              const input = activity.querySelector(`[data-dsa-input="${key}"]`);
              const accepted = acceptedRaw.map(normalizeSymbolic);
              const value = normalizeSymbolic(input?.value);
              return accepted.includes(value) || passesFlexibleIntroCheck(activity, key, input?.value || "");
            });
          });
          if (allPassed) revealComplete("All guided checks are correct. The complete solution is now shown.");
        });
        for (const key of Object.keys(targets)) {
          const input = activity.querySelector(`[data-dsa-input="${key}"]`);
          if (!input) continue;
          input.addEventListener("input", () => {
            input.classList.remove("is-correct", "is-wrong");
            if (feedback) {
              feedback.textContent = "";
              feedback.classList.remove("is-correct", "is-wrong");
            }
            if (answer) answer.hidden = true;
          });
        }
      }
      activity.querySelector("[data-dsa-reveal-all]")?.addEventListener("click", () => {
        for (const answer of activity.querySelectorAll("[data-dsa-answer]")) {
          answer.hidden = false;
          typesetMath(answer);
        }
        revealComplete("Complete solution revealed. Use it to compare against your work.");
      });
    }
  };

  const initQuizChecks = () => {
    const normalizeQuizMarkup = () => {
      for (const quiz of document.querySelectorAll(".quiz-card")) {
        const kind = quiz.dataset.quizType || "mcq";
        if (kind === "mcq") {
          for (const label of quiz.querySelectorAll("label")) {
            const radio = label.querySelector('input[type="radio"]');
            if (!radio || label.querySelector(".quiz-option-text")) continue;
            const textWrap = document.createElement("span");
            textWrap.className = "quiz-option-text";
            while (radio.nextSibling) textWrap.appendChild(radio.nextSibling);
            label.appendChild(textWrap);
          }
        }
        if (kind === "short") {
          for (const label of quiz.querySelectorAll("label")) {
            const input = label.querySelector('input[type="text"], textarea');
            if (!input || label.querySelector(".quiz-input-label")) continue;
            const labelWrap = document.createElement("span");
            labelWrap.className = "quiz-input-label";
            while (label.firstChild && label.firstChild !== input) {
              labelWrap.appendChild(label.firstChild);
            }
            if (labelWrap.textContent.trim() || labelWrap.children.length) {
              label.insertBefore(labelWrap, input);
            }
          }
        }
      }
    };
    normalizeQuizMarkup();

    const answerBlockFor = (quiz) => quiz.querySelector(".quiz-answer") || quiz.querySelector("details");
    const hideQuizAnswer = (quiz) => {
      const answerBlock = answerBlockFor(quiz);
      if (answerBlock) answerBlock.hidden = true;
    };
    const showQuizAnswer = (quiz) => {
      const answerBlock = answerBlockFor(quiz);
      if (answerBlock) answerBlock.hidden = false;
    };
    const setQuizFeedback = (feedback, ok, base, explainHtml = "") => {
      feedback.classList.remove("is-correct", "is-wrong");
      feedback.classList.add(ok ? "is-correct" : "is-wrong");
      feedback.innerHTML = explainHtml
        ? `${base}<br><span class="quiz-explain-inline">${explainHtml}</span>`
        : base;
    };
    const evaluateQuiz = (quiz, { fromSelection = false } = {}) => {
      const feedback = quiz.querySelector(".quiz-feedback");
      if (!feedback) return;
      const kind = quiz.dataset.quizType || "mcq";
      if (kind === "reflection") {
        const input = quiz.querySelector("textarea");
        if (!input) return;
        const minLength = Number.parseInt(quiz.dataset.minLength || "20", 10);
        const response = String(input.value || "").trim();
        const ok = response.length >= minLength;
        quiz.classList.toggle("is-checked", ok);
        setQuizFeedback(
          feedback,
          ok,
          ok ? "Response recorded." : "Add a little more detail, then click Check again.",
        );
        if (ok) showQuizAnswer(quiz);
        return;
      }

      if (kind === "short") {
        const input = quiz.querySelector('input[type="text"], textarea');
        if (!input) return;
        const numericAnswer = quiz.dataset.answerNumber;
        if (numericAnswer !== undefined) {
          const expectedNumber = Number.parseFloat(numericAnswer);
          const tolerance = Number.parseFloat(quiz.dataset.tolerance || "0");
          const guessNumber = Number.parseFloat(String(input.value || "").trim());
          if (Number.isNaN(guessNumber)) {
            const prompt = fromSelection ? "Enter a numeric answer." : "Enter a numeric answer, then click Check.";
            quiz.classList.remove("is-checked");
            hideQuizAnswer(quiz);
            setQuizFeedback(feedback, false, prompt);
            return;
          }
          const ok =
            Number.isFinite(expectedNumber) &&
            Number.isFinite(tolerance) &&
            Math.abs(guessNumber - expectedNumber) <= Math.abs(tolerance);
          quiz.classList.add("is-checked");
          setQuizFeedback(feedback, ok, ok ? "Correct." : "Not quite.");
          showQuizAnswer(quiz);
          return;
        }
        const expected = (quiz.dataset.answers || "")
          .replace(/\\\|/g, "|")
          .split("|")
          .map((x) => x.trim().toLowerCase())
          .filter(Boolean);
        const guess = String(input.value || "").trim().toLowerCase();
        if (!guess) {
          const prompt = fromSelection ? "Enter an answer." : "Enter an answer, then click Check.";
          quiz.classList.remove("is-checked");
          hideQuizAnswer(quiz);
          setQuizFeedback(feedback, false, prompt);
          return;
        }
        const ok = expected.includes(guess);
        quiz.classList.add("is-checked");
        setQuizFeedback(feedback, ok, ok ? "Correct." : "Not quite.");
        showQuizAnswer(quiz);
        return;
      }

      const selected = quiz.querySelector('input[type="radio"]:checked');
      if (!selected) {
        quiz.classList.remove("is-checked");
        hideQuizAnswer(quiz);
        setQuizFeedback(feedback, false, "Select an option first.");
        return;
      }
      const answer = (quiz.dataset.answer || "").trim();
      const ok = selected.value === answer;
      const selectedKey = `feedback${selected.value.toUpperCase()}`;
      quiz.classList.add("is-checked");
      setQuizFeedback(
        feedback,
        ok,
        ok ? "Correct." : "Not quite.",
        quiz.dataset[selectedKey] || quiz.dataset.feedbackFallback || "",
      );
      showQuizAnswer(quiz);
    };
    const resetQuiz = (quiz) => {
      const feedback = quiz.querySelector(".quiz-feedback");
      for (const r of quiz.querySelectorAll('input[type="radio"]')) r.checked = false;
      const input = quiz.querySelector('input[type="text"], textarea');
      if (input) input.value = "";
      if (feedback) {
        feedback.innerHTML = "";
        feedback.classList.remove("is-correct", "is-wrong");
      }
      quiz.classList.remove("is-checked");
      hideQuizAnswer(quiz);
    };
    const quizzes = Array.from(document.querySelectorAll(".quiz-card")).filter(
      (quiz) => !quiz.matches("[data-ch9-final-value-table]"),
    );
    for (const quiz of quizzes) {
      const checkBtn = quiz.querySelector(".quiz-check");
      const resetBtn = quiz.querySelector(".quiz-reset");
      const feedback = quiz.querySelector(".quiz-feedback");
      if (!feedback) continue;
      const kind = quiz.dataset.quizType || "mcq";
      const answerBlock = quiz.querySelector(".quiz-answer") || quiz.querySelector("details");
      const hideAnswer = () => hideQuizAnswer(quiz);
      const showAnswer = () => showQuizAnswer(quiz);

      const getExplainHtml = () => {
        const details = quiz.querySelector(".quiz-answer details");
        if (details) {
          const p = details.querySelector("p");
          if (p && p.innerHTML) return p.innerHTML;
        }
        const legacy = quiz.querySelector("details > p");
        if (legacy && legacy.innerHTML) return legacy.innerHTML;
        return "";
      };

      if (kind === "mcq") {
        hideAnswer();
      } else {
        hideAnswer();
      }

      const setFeedback = (ok, base, explainHtml = "") => {
        feedback.classList.remove("is-correct", "is-wrong");
        feedback.classList.add(ok ? "is-correct" : "is-wrong");
        feedback.innerHTML = explainHtml
          ? `${base}<br><span class="quiz-explain-inline">${explainHtml}</span>`
          : base;
      };

      const evaluate = ({ fromSelection = false } = {}) => {
        if (kind === "reflection") {
          const input = quiz.querySelector("textarea");
          if (!input) return;
          const minLength = Number.parseInt(quiz.dataset.minLength || "20", 10);
          const response = String(input.value || "").trim();
          const ok = response.length >= minLength;
          quiz.classList.toggle("is-checked", ok);
          setFeedback(
            ok,
            ok ? "Response recorded." : "Add a little more detail, then click Check again.",
          );
          if (ok) showAnswer();
          return;
        }

        if (kind === "short") {
          const input = quiz.querySelector('input[type="text"], textarea');
          if (!input) return;
          const numericAnswer = quiz.dataset.answerNumber;
          if (numericAnswer !== undefined) {
            const expectedNumber = Number.parseFloat(numericAnswer);
            const tolerance = Number.parseFloat(quiz.dataset.tolerance || "0");
            const guessNumber = Number.parseFloat(String(input.value || "").trim());
            if (Number.isNaN(guessNumber)) {
              const prompt = fromSelection ? "Enter a numeric answer." : "Enter a numeric answer, then click Check.";
              quiz.classList.remove("is-checked");
              hideAnswer();
              setFeedback(false, prompt);
              return;
            }
            const ok =
              Number.isFinite(expectedNumber) &&
              Number.isFinite(tolerance) &&
              Math.abs(guessNumber - expectedNumber) <= Math.abs(tolerance);
            quiz.classList.add("is-checked");
            setFeedback(ok, ok ? "Correct." : "Not quite.");
            showAnswer();
            return;
          }
          const expected = (quiz.dataset.answers || "")
            .replace(/\\\|/g, "|")
            .split("|")
            .map((x) => x.trim().toLowerCase())
            .filter(Boolean);
          const guess = String(input.value || "").trim().toLowerCase();
          if (!guess) {
            const prompt = fromSelection ? "Enter an answer." : "Enter an answer, then click Check.";
            quiz.classList.remove("is-checked");
            hideAnswer();
            setFeedback(false, prompt);
            return;
          }
          const ok = expected.includes(guess);
          quiz.classList.add("is-checked");
          setFeedback(ok, ok ? "Correct." : "Not quite.");
          showAnswer();
          return;
        }

        const selected = quiz.querySelector('input[type="radio"]:checked');
        if (!selected) {
          quiz.classList.remove("is-checked");
          setFeedback(false, "Select an option first.");
          return;
        }
        const answer = (quiz.dataset.answer || "").trim();
        const ok = selected.value === answer;
        const selectedKey = `feedback${selected.value.toUpperCase()}`;
        quiz.classList.add("is-checked");
        setFeedback(
          ok,
          ok ? "Correct." : "Not quite.",
          quiz.dataset[selectedKey] || quiz.dataset.feedbackFallback || "",
        );
        showAnswer();
      };

      if (checkBtn) {
        checkBtn.addEventListener("click", () => evaluateQuiz(quiz, { fromSelection: false }));
      }

      const radios = quiz.querySelectorAll('input[type="radio"]');
      for (const radio of radios) radio.addEventListener("change", () => {
        feedback.innerHTML = "";
        feedback.classList.remove("is-correct", "is-wrong");
        quiz.classList.remove("is-checked");
        hideAnswer();
      });

      if (resetBtn) {
        resetBtn.addEventListener("click", () => resetQuiz(quiz));
      }

      const textInputs = quiz.querySelectorAll('input[type="text"], textarea');
      for (const input of textInputs) {
        input.addEventListener("input", () => {
          hideAnswer();
          quiz.classList.remove("is-checked");
        });
      }
    }

    // Fallback delegated handlers for dynamically transformed quiz markup.
    document.addEventListener("change", (event) => {
      const target = event.target;
      if (!(target instanceof Element)) return;
      const quiz = target.closest(".quiz-card");
      if (!quiz) return;
      if (!target.matches('input[type="radio"]')) return;
      const feedback = quiz.querySelector(".quiz-feedback");
      if (feedback) {
        feedback.innerHTML = "";
        feedback.classList.remove("is-correct", "is-wrong");
      }
      quiz.classList.remove("is-checked");
    });
    document.addEventListener("click", (event) => {
      const target = event.target;
      if (!(target instanceof Element)) return;
      const button = target.closest(".quiz-check, .quiz-reset");
      if (!button) return;
      const quiz = button.closest(".quiz-card");
      if (!quiz || quiz.matches("[data-ch9-final-value-table]")) return;
      if (button.matches(".quiz-check")) evaluateQuiz(quiz, { fromSelection: false });
      if (button.matches(".quiz-reset")) resetQuiz(quiz);
    });
  };

  fixEscapedAssetPaths();
  initQuizChecks();
  initChapterOneWorkflow();
  initChapterOneDisturbance();
  initChapterFourTaylorExplorer();
  initChapterNineActivities();
  initAdvancedChapterActivities();
  initGuidedDesignActivities();
  initDesignStudyAActivities();
  if (!navWrap || !toc) return;

  if (window.matchMedia("(max-width: 980px)").matches) {
    navWrap.classList.add("is-collapsed");
  }

  if (toggle) {
    toggle.addEventListener("click", () => {
      const collapsed = navWrap.classList.toggle("is-collapsed");
      toggle.setAttribute("aria-expanded", String(!collapsed));
    });
  }

  const links = Array.from(toc.querySelectorAll('a[href^="#"]'));
  const map = new Map();
  for (const a of links) {
    const id = a.getAttribute("href").slice(1);
    const el = document.getElementById(id);
    if (el) map.set(id, { el, a });
  }

  const setActive = (id) => {
    for (const { a } of map.values()) a.classList.remove("is-active");
    const item = map.get(id);
    if (item) item.a.classList.add("is-active");
  };

  const observed = Array.from(map.values()).map((v) => v.el);
  if (observed.length) {
    const observer = new IntersectionObserver(
      (entries) => {
        let best = null;
        for (const e of entries) {
          if (!e.isIntersecting) continue;
          if (!best || e.intersectionRatio > best.intersectionRatio) best = e;
        }
        if (!best) return;
        setActive(best.target.id);
      },
      {
        rootMargin: "-20% 0px -70% 0px",
        threshold: [0.05, 0.2, 0.4],
      },
    );
    for (const el of observed) observer.observe(el);
  }
})();
