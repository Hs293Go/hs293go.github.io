---
layout: home
title: "H S Helson Go's research and notes"
author_profile: false
sidebar:
  nav: "main"
---

Welcome! This is my collection of research notes, technical documentation, and
programming insights from my PhD in robotics and control systems.

The site is divided into three parts:

- [**About Me**](/about/)
- [**My Research**](/research/)
- [**My Notes**](/notes/)

---

### 🆕 Recent Posts

{% for post in site.posts limit:5 %}

- [{{ post.title }}]({{ post.url | relative_url }}) —
  {{ post.date | date: "%b %d, %Y" }} {% endfor %}
