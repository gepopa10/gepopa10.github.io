# Building a george Blog with GitHub Pages, Jekyll, and the Chirpy Theme

Creating a personal blog to showcase your technical content doesn't have to be complicated. With tools like GitHub Pages, the Jekyll CMS, and the feature-rich Chirpy theme, you can build and deploy a fast, elegant blog site — with zero hosting cost.

This guide walks through the implementation, explains lesser-known caveats, and gives practical advice from real experience. We'll also use Visual Studio Code for editing and previewing posts in Markdown.

For reference, the official documentation for the Chirpy theme is available at the [Chirpy Demo Site](https://chirpy.cotes.page/). It's a great starting point to understand the layout, features, and configuration options of the theme.

---

## Why Static Blogs?

Static blogs are:

- **Free to host** (especially on GitHub Pages)
- **Fast and secure**: no backend = fewer risks
- **Portable and version-controlled**
- **Great for developers**: each post is just a Markdown file

Unlike dynamic sites, static blogs don't require a server-side engine like PHP or Node.js. Instead, they're compiled once (during the deployment process) into static HTML, CSS, and JS files that are then served directly by GitHub Pages. This means faster load times, reduced attack surface, and no database to manage.

---

## Tech Stack Overview

| Tool | Role |
|---|---|
| **GitHub Pages** | Free static hosting for Jekyll sites |
| **Jekyll** | Static site generator based on Ruby |
| **Chirpy Theme** | A modern Jekyll theme tailored for technical blogging |
| **Visual Studio Code** | Editor for writing and managing the site locally |

---

## Setting It Up from Scratch

### 1. Clone the Chirpy Starter Template

Start from the [Chirpy Starter Theme](https://github.com/cotes2020/chirpy-starter):

```bash
git clone https://github.com/cotes2020/chirpy-starter.git my-blog
cd my-blog
```

This starter theme is a lightweight fork of the full Chirpy theme, created for easier deployment with GitHub Pages. It exposes only the parts you'll likely need to customize, while hiding the internals that are managed via a RubyGem.

---

## Understanding the Chirpy Ecosystem

There are actually **three repositories** in the Chirpy ecosystem:

**`jekyll-theme-chirpy`**
The full theme codebase packaged as a RubyGem. This is the "engine" behind the blog but not meant to be forked directly.

**`chirpy-starter`**
A minimal repo that uses the theme as a dependency, ideal for quick deployment and minimal customization.

**`chirpy-static-assets`**
A separate repository containing the actual JS/CSS/font assets needed by the theme — especially if you choose to self-host them instead of relying on external CDNs.

> **Important Note:** In practice, I wasn't able to make my site fully functional without integrating this third repo. Some scripts and stylesheets never loaded correctly from the CDN, despite the documentation suggesting that no extra steps were required.
>
> My workaround was to download the static assets locally and add them as a Git submodule under `assets/lib`. The static assets repo provides clear instructions on how to do this. Without it, several layout and JavaScript features failed silently.

---

## GitHub Integration and Deployment

**Link your repo:**
```bash
git remote add origin https://github.com/youruser/yourblog.git
```

**Push your work:**
```bash
git add .
git commit -m "Initial commit"
git push -u origin main
```

**Enable GitHub Pages:**
In your repo settings, go to **Pages**, and set the source to **GitHub Actions** (not the branch directly). This ensures that each time you push changes, GitHub runs a build pipeline to generate the static HTML files and publish them.

This mechanism is what transforms your Markdown and configuration files into a fully-rendered static website hosted for free.

> **Keep in mind:** Since GitHub Pages builds the site for you, your source files must conform to GitHub's current rules. In my case, a build started failing because some links used `http://` instead of `https://`. Although this had worked previously, GitHub changed their rules, and new validations were enforced. Always check your **GitHub Actions logs** to catch these issues early.

---

## Post Structure in Jekyll + Chirpy

Each post in Jekyll starts with a **front matter** — a YAML section enclosed by `---` that contains metadata about the post. This metadata guides how Jekyll processes and displays the content.

**Example:**

```yaml
---
title: "My Awesome Post"
date: 2025-06-17 10:00:00 +0000
categories: [Azure, AI, LLM]
tags: [cloud, github]
layout: post
---
```

After this front matter, the post body follows in Markdown format.

---

## Category System in Chirpy

- The `categories` field accepts an array.
- The **first entry** defines the top-level category.
- **Subsequent entries** define nested subcategories.

**Example:**

```yaml
categories: [Azure, AI, LLM]
```

This will produce:

```
/categories/azure/
/categories/azure/ai/
/categories/azure/ai/llm/
```

This hierarchy is reflected in the archive navigation and helps structure your content cleanly.

> **Tip:** Stick to lowercase and hyphenated names to ensure clean URLs.

---

## Writing Posts in Visual Studio Code

Open the repo in VS Code. Inside the `_posts/` folder, create your post like:

```
_posts/2025-06-17-my-awesome-post.md
```

Then write your content in Markdown. You can use VS Code's **Markdown preview** feature to see how your post will render.

---

## Local Preview

To preview your site locally:

```bash
bundle exec jekyll serve
```

Open your browser at `http://localhost:4000`. You'll see your site exactly as it will be rendered on GitHub Pages.

---

## Final Thoughts

Deploying a Chirpy-themed blog with GitHub Pages and Jekyll is a fantastic way to start writing and sharing technical content. With a bit of setup and patience — especially when handling static assets and submodules — you get a solid, professional blog platform that's fast, free, and customizable.

Just stay aware that GitHub may update its rules for site builds. Always test and review **GitHub Actions logs** after pushes, especially when adding new content or adjusting your theme.
