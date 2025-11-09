# Build Instructions

## Local preview

1. Install Ruby (>= 3.0) and Bundler.
2. Run:

   ```bash
   bundle install
   bundle exec jekyll serve
   ```

3. Open [http://localhost:4000](http://localhost:4000) to preview.

## Deploy to GitHub Pages

1. Create a repo named `<username>.github.io` or any project repo.
2. Commit this directory's contents and push to `main`.
3. On GitHub → **Settings → Pages**, set source: `main` branch, root directory.
4. Pages will automatically build with GitHub’s built-in Jekyll engine.

You now have a two-layer site with sidebar navigation for:

- About Me / CV
- Research
- Tech Insights (posts)
