(() => {
  const navWrap = document.querySelector(".site-nav");
  const toc = document.getElementById("TOC");
  const toggle = document.querySelector(".toc-toggle");
  const initQuizChecks = () => {
    const quizzes = Array.from(document.querySelectorAll(".quiz-card"));
    for (const quiz of quizzes) {
      const checkBtn = quiz.querySelector(".quiz-check");
      const resetBtn = quiz.querySelector(".quiz-reset");
      const feedback = quiz.querySelector(".quiz-feedback");
      if (!feedback) continue;

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

      const setFeedback = (ok, base, explainHtml = "") => {
        feedback.classList.remove("is-correct", "is-wrong");
        feedback.classList.add(ok ? "is-correct" : "is-wrong");
        feedback.innerHTML = explainHtml
          ? `${base}<br><span class="quiz-explain-inline">${explainHtml}</span>`
          : base;
      };

      const evaluate = ({ fromSelection = false } = {}) => {
        const kind = quiz.dataset.quizType || "mcq";
        const explainHtml = getExplainHtml();

        if (kind === "short") {
          const input = quiz.querySelector('input[type="text"], textarea');
          if (!input) return;
          const expected = (quiz.dataset.answers || "")
            .split("|")
            .map((x) => x.trim().toLowerCase())
            .filter(Boolean);
          const guess = String(input.value || "").trim().toLowerCase();
          if (!guess) {
            const prompt = fromSelection ? "Enter an answer." : "Enter an answer, then click Check.";
            setFeedback(false, prompt);
            return;
          }
          const ok = expected.includes(guess);
          setFeedback(ok, ok ? "Correct." : "Not quite.", explainHtml);
          return;
        }

        const selected = quiz.querySelector('input[type="radio"]:checked');
        if (!selected) {
          setFeedback(false, "Select an option first.");
          return;
        }
        const answer = (quiz.dataset.answer || "").trim();
        const ok = selected.value === answer;
        setFeedback(ok, ok ? "Correct." : "Not quite.", explainHtml);
      };

      if (checkBtn) {
        checkBtn.addEventListener("click", () => evaluate({ fromSelection: false }));
      }

      const radios = quiz.querySelectorAll('input[type="radio"]');
      for (const radio of radios) {
        radio.addEventListener("change", () => evaluate({ fromSelection: true }));
      }

      const input = quiz.querySelector('input[type="text"], textarea');
      if (input) {
        input.addEventListener("change", () => evaluate({ fromSelection: true }));
      }

      if (resetBtn) {
        resetBtn.addEventListener("click", () => {
          const radios = quiz.querySelectorAll('input[type="radio"]');
          for (const r of radios) r.checked = false;
          const input = quiz.querySelector('input[type="text"], textarea');
          if (input) input.value = "";
          feedback.innerHTML = "";
          feedback.classList.remove("is-correct", "is-wrong");
        });
      }
    }

    // Fallback delegated handlers for dynamically transformed quiz markup.
    document.addEventListener("change", (event) => {
      const target = event.target;
      if (!(target instanceof Element)) return;
      const quiz = target.closest(".quiz-card");
      if (!quiz) return;
      if (!target.matches('input[type="radio"], input[type="text"], textarea')) return;
      const check = quiz.querySelector(".quiz-check");
      if (check) check.click();
    });

    document.addEventListener("input", (event) => {
      const target = event.target;
      if (!(target instanceof Element)) return;
      if (!target.matches('.quiz-card input[type="text"], .quiz-card textarea')) return;
      const quiz = target.closest(".quiz-card");
      if (!quiz) return;
      const check = quiz.querySelector(".quiz-check");
      if (check) check.click();
    });
  };

  initQuizChecks();
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
