import comp from "E:/lab/zhize-web-main/my-docs/src/.vuepress/.temp/pages/demo/Members/index.html.vue"
const data = JSON.parse("{\"path\":\"/demo/Members/\",\"title\":\"成员介绍\",\"lang\":\"zh-CN\",\"frontmatter\":{\"title\":\"成员介绍\",\"icon\":\"users\",\"category\":[\"关于实验室\"],\"description\":\"暂空\",\"head\":[[\"script\",{\"type\":\"application/ld+json\"},\"{\\\"@context\\\":\\\"https://schema.org\\\",\\\"@type\\\":\\\"Article\\\",\\\"headline\\\":\\\"成员介绍\\\",\\\"image\\\":[\\\"\\\"],\\\"dateModified\\\":null,\\\"author\\\":[{\\\"@type\\\":\\\"Person\\\",\\\"name\\\":\\\"Mr.Hope\\\",\\\"url\\\":\\\"https://mister-hope.com\\\"}]}\"],[\"meta\",{\"property\":\"og:url\",\"content\":\"https://vuepress-theme-hope-docs-demo.netlify.app/demo/Members/\"}],[\"meta\",{\"property\":\"og:site_name\",\"content\":\"智泽实验室\"}],[\"meta\",{\"property\":\"og:title\",\"content\":\"成员介绍\"}],[\"meta\",{\"property\":\"og:description\",\"content\":\"暂空\"}],[\"meta\",{\"property\":\"og:type\",\"content\":\"article\"}],[\"meta\",{\"property\":\"og:locale\",\"content\":\"zh-CN\"}]]},\"readingTime\":{\"minutes\":0.05,\"words\":15},\"filePathRelative\":\"demo/Members/README.md\",\"autoDesc\":true}")
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
