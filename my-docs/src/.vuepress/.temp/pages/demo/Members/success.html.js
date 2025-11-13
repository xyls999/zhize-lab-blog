import comp from "E:/lab/zhize-web-main/my-docs/src/.vuepress/.temp/pages/demo/Members/success.html.vue"
const data = JSON.parse("{\"path\":\"/demo/Members/success.html\",\"title\":\"成功展示\",\"lang\":\"zh-CN\",\"frontmatter\":{\"title\":\"成功展示\",\"icon\":\"trophy\",\"order\":1,\"category\":[\"关于实验室\"],\"description\":\"成功展示 竞赛获奖 项目成果 学术成果\"},\"readingTime\":{\"minutes\":0.28,\"words\":84},\"filePathRelative\":\"demo/Members/success.md\",\"autoDesc\":true}")
export { comp, data }

if (import.meta.webpackHot) {
  import.meta.webpackHot.accept()
  if (__VUE_HMR_RUNTIME__.updatePageData) {
    __VUE_HMR_RUNTIME__.updatePageData(data)
  }
}

if (import.meta.hot) {
  import.meta.hot.accept(({ data }) => {
    __VUE_HMR_RUNTIME__.updatePageData(data)
  })
}
