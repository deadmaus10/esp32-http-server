<?php
declare(strict_types=1);

function renderLandingPage(string $contactEmail, string $cssHref): void
{
    $emailEsc = htmlspecialchars($contactEmail, ENT_QUOTES, 'UTF-8');
    $cssHrefEsc = htmlspecialchars($cssHref, ENT_QUOTES, 'UTF-8');

    $html = <<<HTML
<!doctype html>
<html lang="hu">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="description" content="Az Alba Squeeze Solution Kft. weboldala hamarosan érkezik.">
  <meta name="theme-color" content="#071829">
  <title>Alba Squeeze Solution Kft. — Hamarosan</title>
  <link rel="stylesheet" href="{$cssHrefEsc}">
</head>
<body class="landing-page">
  <main class="landing-shell">
    <header class="landing-header">
      <a class="wordmark" href="#top" aria-label="Alba Squeeze Solution Kft. – kezdőlap">
        <span class="wordmark-mark" aria-hidden="true"></span>
        <span>Alba Squeeze<br class="wordmark-break"> Solution Kft.</span>
      </a>
      <a class="contact-link" href="mailto:{$emailEsc}">Kapcsolat</a>
    </header>

    <section class="hero" id="top" aria-labelledby="hero-title">
      <div class="hero-copy">
        <h1 id="hero-title">Valami új<br>formát ölt.</h1>
        <p>Az Alba Squeeze Solution Kft.<br class="desktop-break"> weboldala hamarosan érkezik.</p>
        <a class="email-button" href="mailto:{$emailEsc}">
          <svg aria-hidden="true" viewBox="0 0 24 24" fill="none">
            <path d="M3.75 6.75h16.5v10.5H3.75z" />
            <path d="m4.5 7.5 7.5 6 7.5-6" />
          </svg>
          <span>Írjon nekünk</span>
        </a>
      </div>

      <div class="squeeze-art" aria-hidden="true">
        <div class="shape shape-top"></div>
        <div class="shape shape-bottom"></div>
        <svg class="flow-lines" viewBox="0 0 680 760" preserveAspectRatio="none">
          <g>
            <path d="M4 318 C190 318 227 377 340 380 C453 383 492 318 676 318" />
            <path d="M4 330 C184 330 225 376 340 380 C455 384 496 330 676 330" />
            <path d="M4 342 C178 342 223 376 340 380 C457 384 502 342 676 342" />
            <path d="M4 354 C172 354 221 377 340 380 C459 383 508 354 676 354" />
            <path d="M4 366 C166 366 219 378 340 380 C461 382 514 366 676 366" />
            <path d="M4 378 C160 378 217 379 340 380 C463 381 520 378 676 378" />
            <path d="M4 390 C160 390 217 381 340 380 C463 379 520 390 676 390" />
            <path d="M4 402 C166 402 219 382 340 380 C461 378 514 402 676 402" />
            <path d="M4 414 C172 414 221 383 340 380 C459 377 508 414 676 414" />
            <path d="M4 426 C178 426 223 384 340 380 C457 376 502 426 676 426" />
            <path d="M4 438 C184 438 225 384 340 380 C455 376 496 438 676 438" />
            <path d="M4 450 C190 450 227 383 340 380 C453 377 492 450 676 450" />
          </g>
        </svg>
      </div>
    </section>

    <footer class="landing-footer">
      <span>Alba Squeeze Solution Kft.</span>
      <span>Magyarország</span>
      <a href="mailto:{$emailEsc}">{$emailEsc}</a>
    </footer>
  </main>
</body>
</html>
HTML;

    respondHtml(200, $html);
}
