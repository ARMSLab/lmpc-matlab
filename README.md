<script type="text/javascript" async
  src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.2/MathJax.js?config=TeX-MML-AM_CHTML">
</script>

## Welcome to GitHub Pages

You can use the [editor on GitHub](https://github.com/ARMSLab/lmpc-matlab/edit/master/README.md) to maintain and preview the content for your website in Markdown files.

Whenever you commit to this repository, GitHub Pages will run [Jekyll](https://jekyllrb.com/) to rebuild the pages in your site, from the content in your Markdown files.

### Markdown

Markdown is a lightweight and easy-to-use syntax for styling your writing. It includes conventions for

```markdown
Syntax highlighted code block

# Header 1
## Header 2
### Header 3

- Bulleted
- List

1. Numbered
2. List

**Bold** and _Italic_ and `Code` text

[Link](url) and ![Image](src)
```

For more details see [GitHub Flavored Markdown](https://guides.github.com/features/mastering-markdown/).

### Jekyll Themes

Your Pages site will use the layout and styles from the Jekyll theme you have selected in your [repository settings](https://github.com/ARMSLab/lmpc-matlab/settings). The name of this theme is saved in the Jekyll `_config.yml` configuration file.

### Support or Contact

Having trouble with Pages? Check out our [documentation](https://help.github.com/categories/github-pages-basics/) or [contact support](https://github.com/contact) and we’ll help you sort it out.

# Tutorial on Linearized MPC controller


This is tutorial of LMPC controller for nonlinear system. In this tutorial basic parts of Linearized MPC controller would be examined. This script shows how to implement controller for nonlinear system provided by equation **dx/dt = f(x,u)** and **y=C*x+D*u**. For details of derivation please refer to  [Zhakatayev, Altay, et al. "Successive linearization based model predictive control of variable stiffness actuated robots." IEEE AIM 2017](http://ieeexplore.ieee.org/document/8014275/)

linearize, discretize and how to find solution with simple constraints are shown. We setup out tutorial on simple mathematical pendulum, with equation of motion:

$$\frac{\partial x_1}{\partial t} = x_2 $$ ,

$$\frac{\partial x_2}{\partial t} = -\frac{g}{l} sin(x_1) -b*x_2 + u $$

 Model Predictive Control scheme can be formulated as an optimization problem with horizon length np:
$$ Min J = \frac{1}{2}*(X-rr)^T*Q*(X-rr) + \frac{1}{2}u^T*R*u = \frac{1}{2}*u^TG*u + f^T*u + C $$ 
