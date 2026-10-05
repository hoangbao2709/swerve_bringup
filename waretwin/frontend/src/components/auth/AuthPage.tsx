import { useState } from "react";
import { login } from "../../services/auth";

function go(path: string) {
  window.history.pushState({}, "", path);
  window.dispatchEvent(new PopStateEvent("popstate"));
}

export function AuthPage() {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showPassword, setShowPassword] = useState(false);

  const submit = async (event: React.FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    if (!username.trim() || !password) {
      setError("Enter your username and password.");
      return;
    }

    setBusy(true);
    setError(null);
    try {
      await login(username.trim(), password);
      go("/");
    } catch (reason) {
      setError(reason instanceof Error ? reason.message.slice(0, 200) : "Login failed. Check your connection and credentials.");
    } finally {
      setBusy(false);
    }
  };

  return (
    <main className="auth-page">
      <div className="auth-grid" />
      <div className="auth-glow auth-glow-left" />
      <div className="auth-glow auth-glow-right" />
      <div className="auth-container">
        <section className="auth-intro">
          <div className="auth-brand">
            <div className="auth-logo"><span className="logo-mark">P</span></div>
            <div>
              <div className="brand-name"><span>PTAGV</span> / WareTwin</div>
              <div className="brand-caption">ROBOT CONTROL</div>
            </div>
          </div>
          <div className="auth-headline">
            <div className="eyebrow"><span className="status-dot" /> CONTROL SYSTEM ACCESS</div>
            <h2>Robot Control</h2>
            <p>Sign in to view live robot state and control the ROS 2 runtime.</p>
          </div>
          <div className="auth-system-status">
            <div className="system-status-label"><span className="status-dot" /> BACKEND AUTHENTICATION REQUIRED</div>
            <div className="system-status-items"><span>DJANGO</span><span>ROS 2</span><span>GAZEBO</span></div>
          </div>
        </section>

        <section className="auth-panel">
          <div className="auth-card">
            <div className="auth-card-header">
              <div className="auth-card-icon">→</div>
              <div>
                <div className="auth-card-kicker">PTAGV / WARETWIN</div>
                <h1>Sign in</h1>
                <p>Use your backend account to open Robot Control.</p>
              </div>
            </div>

            <form className="auth-form" onSubmit={submit}>
              <label className="auth-field">
                <span>Username</span>
                <div className="input-wrap">
                  <span className="input-icon">◎</span>
                  <input value={username} onChange={(event) => setUsername(event.target.value)}
                    autoComplete="username" placeholder="Enter your username" disabled={busy} />
                </div>
              </label>

              <label className="auth-field">
                <span>Password</span>
                <div className="input-wrap">
                  <span className="input-icon">◇</span>
                  <input type={showPassword ? "text" : "password"} value={password}
                    onChange={(event) => setPassword(event.target.value)} autoComplete="current-password"
                    placeholder="Enter your password" disabled={busy} />
                  <button type="button" className="password-toggle" onClick={() => setShowPassword((value) => !value)} tabIndex={-1}>
                    {showPassword ? "Hide" : "Show"}
                  </button>
                </div>
              </label>

              {error && <div className="form-err" role="alert"><span>!</span><span>{error}</span></div>}
              <button className="auth-submit" type="submit" disabled={busy}>
                <span>{busy ? "Authenticating…" : "Sign in to Robot Control"}</span>
                {!busy && <span className="submit-arrow">→</span>}
              </button>
            </form>

            <div className="auth-security">
              <span className="security-icon">✓</span>
              <span>Backend verified session · Protected control channel</span>
            </div>
          </div>
          <div className="auth-footer"><span>PTAGV / WareTwin</span></div>
        </section>
      </div>
    </main>
  );
}
