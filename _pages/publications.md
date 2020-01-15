---
layout: archive
title: "Publications"
permalink: /publications/
author_profile: true
---
[Maximal Information Propagation with Budgets](https://arxiv.org/abs/1912.04056)
=====
*Haomin Shi*, Yao Zhang, Zilin Si, Letong Wang and Dengji Zhao, 
Accepted by ECAI 2020.

[Deep-SLAM++ Object-level RGBD SLAM based on Class-specific Deep Shape Priors](https://arxiv.org/abs/1907.09691)
=====
Lan Hu, Wanting Xu, *Haomin Shi*, Kun Huang and Laurent Kneip, 
Submitted to ICRA 2020.

Aligning the impossible: Globally optimal point set registration by joint symmetry plane fitting
======
Lan Hu, *Haomin Shi*, and Laurent Kneip,  
<!-- Submitted to CVPR 2020. -->




{% if author.googlescholar %}
  You can also find my articles on <u><a href="{{author.googlescholar}}">my Google Scholar profile</a>.</u>
{% endif %}

{% include base_path %}

{% for post in site.publications reversed %}
  {% include archive-single.html %}
{% endfor %}
