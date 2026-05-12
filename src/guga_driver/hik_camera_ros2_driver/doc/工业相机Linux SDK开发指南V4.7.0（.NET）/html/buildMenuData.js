var root = document.getElementsByTagName('body')[0]

function findAndInsert(root) {
  if (!root) return
  
  var text, url;
  var children = [];

  for (var i = 0;i < root.children.length;i++) {
    var child = root.children[i]
    if (!child) continue;

    var tag = child.nodeName.toLowerCase()
    switch (tag) {
      case 'object':
        text = child.querySelector('param[name="Name"]')?.value || ''
        url = child.querySelector('param[name="Local"]')?.value || ''
        break;
      case 'ul':
        for (var j = 0;j < child.children.length;j++) {
          children.push(findAndInsert(child.children[j]))
        }
        break;
    }
  }
  return { text: text || undefined, url: url || undefined, children: children.length ? children : undefined }
}

var menudata = findAndInsert(root)
console.log(JSON.stringify(menudata))