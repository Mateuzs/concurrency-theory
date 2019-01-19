package webcrawler

import java.io.File
import java.net.MalformedURLException
import org.htmlcleaner.{CleanerProperties, TagNode}
import scala.concurrent.ExecutionContext.Implicits.global
import scala.concurrent.duration._
import scala.concurrent.{Await, Future}
import scala.io.Source
import scala.util.{Success, Failure}

// traverse na wiele future , sequence


object WebCrawler extends App {

  import java.net.URL
  import org.htmlcleaner.HtmlCleaner
  val cleaner: HtmlCleaner = new HtmlCleaner
  val props: CleanerProperties = cleaner.gerutProperties

  val awaitMax: Duration = 100 second

  def download(url: URL, deep: Int): Future[Boolean] = Future {
    println(deep + " " + url)
    if (deep == 0) {
      true
    }
    else {
      try {
        extractElements(url)
          .map(element => convertToUrl(element.getAttributeByName("href")))
          .filter(url => url.isDefined)
          .map(url => {
            savePage(url.get)
            download(url.get, deep - 1)
          })
          .foreach(future => Await.result(future, awaitMax))
      }
      catch {
        case e: MalformedURLException => println("MALFORMEND URL")
        case x: Exception => print(x)
      }
      true
    }
  }

  def convertToUrl(url: String): Option[URL] = Option {
    try {
      new URL(url)
    }
    catch {
      case e: MalformedURLException => null
    }
  }

  def extractElements(url: URL): Array[TagNode] = {
    val rootNode: TagNode = cleaner.clean(url)
    rootNode.getElementsByName("a", true)
  }

  def savePage(url: URL): Future[Unit] = Future{
    import java.io.BufferedWriter
    import java.io.FileWriter
    import java.io.IOException
    val dirName = "download/" + url.toString
      .replace("http://", "")
      .replace("https://", "")
      .replace(".", "/")
    val fileName = "index.html"

    val dir: File = new File(dirName)
    if (!dir.exists) {
      dir.mkdirs
    }

    val file: File = new File(dirName + "/" + fileName)
    try {
      val fileWriter = new FileWriter(file.getAbsoluteFile)
      val bufferedWriter = new BufferedWriter(fileWriter)
      val html = Source.fromURL(url)
      bufferedWriter.write(html.mkString)
      bufferedWriter.close()
    } catch {
      case e: IOException => println("IO exception: " + e)
      case x: Exception => println(x)
    }
  }

  val googleUrl: URL = new URL("https://www.google.com/")
  val result: Future[Boolean] = download(googleUrl, 2)
  
   println("Starting...")

  result onComplete {
    case Success(arr) => println("\nDone!")
    case Failure(err) => println("Something went wrong! Error message " + err.getMessage)
  }
  
}