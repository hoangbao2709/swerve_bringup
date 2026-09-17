import { useState } from "react";
import { login, register as registerUser } from "../../services/auth";

function go(path: string) {
  window.history.pushState({}, "", path);
  window.dispatchEvent(new PopStateEvent("popstate"));
}

export function AuthPage({ mode }: { mode: "login" | "register" }) {
  const [username, setUsername] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showPassword, setShowPassword] = useState(false);

  const submit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!username.trim() || !password) {
      setError("Please enter your username and password.");
      return;
    }

    if (mode === "register" && !email.trim()) {
      setError("Please enter your email address.");
      return;
    }

    setBusy(true);
    setError(null);

    try {
      if (mode === "login") {
        await login(username.trim(), password);
        go("/");
      } else {
        await registerUser(
          username.trim(),
          email.trim(),
          password
        );
        go("/login");
      }
    } catch (err) {
      setError((err as Error).message.slice(0, 200));
    } finally {
      setBusy(false);
    }
  };

  const isLogin = mode === "login";

  return (
    <div className="auth-page">
      {/* Background grid */}
      <div className="auth-grid" />

      {/* Ambient lights */}
      <div className="auth-glow auth-glow-left" />
      <div className="auth-glow auth-glow-right" />

      <div className="auth-container">

        {/* LEFT — BRAND / SYSTEM INTRO */}
        <section className="auth-intro">
          <div className="auth-brand">
            <div className="auth-logo">
              <span className="logo-mark">W</span>
            </div>

            <div>
              <div className="brand-name">
                <span>Ware</span>Twin
              </div>
              <div className="brand-caption">
                WAREHOUSE DIGITAL TWIN
              </div>
            </div>
          </div>

          <div className="auth-headline">
            <div className="eyebrow">
              <span className="status-dot" />
              AUTONOMOUS WAREHOUSE SYSTEM
            </div>

            <h2>
              Intelligent control for
              <br />
              <span>your digital warehouse.</span>
            </h2>

            <p>
              Monitor your robot fleet, analyze warehouse operations,
              simulate scenarios and make smarter decisions in real time.
            </p>
          </div>

          <div className="auth-features">
            <div className="feature-item">
              <div className="feature-icon">◈</div>
              <div>
                <strong>Real-time Fleet</strong>
                <span>Monitor autonomous robots in real time.</span>
              </div>
            </div>

            <div className="feature-item">
              <div className="feature-icon">⌁</div>
              <div>
                <strong>Digital Twin</strong>
                <span>Visualize and simulate warehouse operations.</span>
              </div>
            </div>

            <div className="feature-item">
              <div className="feature-icon">✦</div>
              <div>
                <strong>AI Operations</strong>
                <span>Use AI to analyze events and optimize the fleet.</span>
              </div>
            </div>
          </div>

          <div className="auth-system-status">
            <div className="system-status-label">
              <span className="status-dot" />
              SYSTEM READY
            </div>

            <div className="system-status-items">
              <span>SIMULATION</span>
              <span>WEBSOCKET</span>
              <span>AI ENGINE</span>
            </div>
          </div>
        </section>

        {/* RIGHT — AUTH CARD */}
        <section className="auth-panel">
          <div className="auth-card">

            <div className="auth-card-header">
              <div className="auth-card-icon">
                {isLogin ? "→" : "+"}
              </div>

              <div>
                <div className="auth-card-kicker">
                  WARETWIN CONSOLE
                </div>

                <h1>
                  {isLogin ? "Welcome back" : "Create your account"}
                </h1>

                <p>
                  {isLogin
                    ? "Sign in to access your warehouse console."
                    : "Create an account to access the simulation console."}
                </p>
              </div>
            </div>

            <form className="auth-form" onSubmit={submit}>

              <label className="auth-field">
                <span>Username</span>

                <div className="input-wrap">
                  <span className="input-icon">◎</span>

                  <input
                    value={username}
                    onChange={(e) => setUsername(e.target.value)}
                    autoComplete="username"
                    placeholder="Enter your username"
                    disabled={busy}
                  />
                </div>
              </label>

              {!isLogin && (
                <label className="auth-field">
                  <span>Email address</span>

                  <div className="input-wrap">
                    <span className="input-icon">@</span>

                    <input
                      type="email"
                      value={email}
                      onChange={(e) => setEmail(e.target.value)}
                      autoComplete="email"
                      placeholder="you@example.com"
                      disabled={busy}
                    />
                  </div>
                </label>
              )}

              <label className="auth-field">
                <span>Password</span>

                <div className="input-wrap">
                  <span className="input-icon">◇</span>

                  <input
                    type={showPassword ? "text" : "password"}
                    value={password}
                    onChange={(e) => setPassword(e.target.value)}
                    autoComplete={
                      isLogin
                        ? "current-password"
                        : "new-password"
                    }
                    placeholder="Enter your password"
                    disabled={busy}
                  />

                  <button
                    type="button"
                    className="password-toggle"
                    onClick={() => setShowPassword((v) => !v)}
                    tabIndex={-1}
                  >
                    {showPassword ? "Hide" : "Show"}
                  </button>
                </div>
              </label>

              {error && (
                <div className="form-err">
                  <span>!</span>
                  <span>{error}</span>
                </div>
              )}

              <button
                className="auth-submit"
                type="submit"
                disabled={busy}
              >
                <span>
                  {busy
                    ? "Authenticating..."
                    : isLogin
                      ? "Sign in to console"
                      : "Create account"}
                </span>

                {!busy && <span className="submit-arrow">→</span>}
              </button>
            </form>

            <div className="auth-divider">
              <span>ACCOUNT</span>
            </div>

            <button
              className="auth-switch"
              onClick={() =>
                go(isLogin ? "/register" : "/login")
              }
            >
              {isLogin ? (
                <>
                  Don't have an account?
                  <strong>Create one</strong>
                </>
              ) : (
                <>
                  Already have an account?
                  <strong>Sign in</strong>
                </>
              )}
            </button>

            <div className="auth-security">
              <span className="security-icon">✓</span>
              <span>
                Secure authentication · Protected session
              </span>
            </div>

          </div>

          <div className="auth-footer">
            <span>WareTwin</span>
            <span>•</span>
            <span>Warehouse Digital Twin Platform</span>
          </div>
        </section>
      </div>
    </div>
  );
}
