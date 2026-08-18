# GitHub + Streamlit Deployment Checklist

## A. Create the GitHub repository

1. Sign in to GitHub.
2. Click the **+** menu in the upper-right corner and choose **New repository**.
3. Suggested repository name: `directional-drilling-autonomy`.
4. Description: `Reduced-order directional drilling simulation with constrained MPC and supervisory hazard avoidance.`
5. Choose **Public** if you want hiring managers to open it without access permissions.
6. Do **not** initialize the repository with a README, `.gitignore`, or license because those files are already included in this project package.
7. Click **Create repository**.

## B. Upload the project

1. Extract the project ZIP on your computer.
2. In the new GitHub repository, click **uploading an existing file** or **Add file → Upload files**.
3. Upload the contents of the extracted folder so that `app.py`, `README.md`, and `requirements.txt` are visible at the repository root.
4. Ensure the `.streamlit` directory is also uploaded and contains `config.toml`.
5. Commit the upload to the `main` branch.

Expected repository root:

```text
app.py
physics_engine.py
control_theory.py
agent_orchestrator.py
requirements.txt
README.md
DEPLOYMENT.md
.gitignore
.streamlit/config.toml
```

## C. Deploy to Streamlit Community Cloud

1. Open Streamlit Community Cloud and sign in with GitHub.
2. Choose **Create app**.
3. Select the GitHub repository you just created.
4. Branch: `main`.
5. Main file path: `app.py`.
6. Open **Advanced settings** and select Python 3.12 or newer.
7. Choose an app subdomain if desired.
8. Click **Deploy**.

## D. After deployment

1. Run the default Agentic Override scenario.
2. Reset and run Legacy Automation Mode.
3. Confirm the plot, metrics, audit terminal, and ZIP download button all render.
4. Copy the public Streamlit URL.
5. Add the URL to the GitHub repository's **About** section under **Website**.
6. Add a short repository topic set such as: `streamlit`, `model-predictive-control`, `directional-drilling`, `automation`, `autonomous-systems`, `python`.

## E. Recommended GitHub About text

**Description**  
`Reduced-order directional drilling simulation with constrained MPC, structural-health telemetry, and supervisory autonomous hazard avoidance.`

**Website**  
Use the public `*.streamlit.app` URL after deployment.

## F. Important public-facing disclaimer

Keep the README disclaimer intact. This project is an independent educational/R&D demonstration and should not be represented as a Halliburton product, a reproduction of proprietary controls, or a field-calibrated drilling simulator.
